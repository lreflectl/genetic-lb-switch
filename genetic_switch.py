from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_0
from ryu.topology.api import get_switch, get_link
from ryu.lib.mac import haddr_to_bin
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet
from ryu.lib.packet import ether_types
from utils import output_packet_port, install_ports_to_path

import sys
import os
# Import modules from another neighbour repository (genetic-pathfinder)
sys.path.append(os.path.dirname(os.path.expanduser('~/py/genetic-pathfinder/genetic_algorithm.py')))
sys.path.append(os.path.dirname(os.path.expanduser('~/py/genetic-pathfinder/baseline_algorithms.py')))
sys.path.append(os.path.dirname(os.path.expanduser('~/py/genetic-pathfinder/stp_algorithm.py')))
sys.path.append(os.path.dirname(os.path.expanduser('~/py/genetic-pathfinder/python_graph.py')))
from baseline_algorithms import dijkstra
from python_graph import Graph
from genetic_algorithm import genetic
from stp_algorithm import build_link_tree, build_spanning_tree
from stp_algorithm import remove_cycles, remove_identical_links, set_reverse_links


class GLBSwitch(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_0.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(GLBSwitch, self).__init__(*args, **kwargs)
        self.mac_to_switch_port = {}
        self.topology_graph = None
        self.switch_to_idx = {}
        self.switches = []
        self.link_tree = None
        self.spanning_tree = None

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        pkt = packet.Packet(ev.msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        # Ignore LLDPPackets used for topology discovery
        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        dst = eth.dst
        src = eth.src
        dpid = ev.msg.datapath.id
        in_port = ev.msg.in_port

        self.logger.info(f"Packet in port {in_port} of switch {dpid} ({src} to {dst})")

        # Saving only the first switch that has got a packet, as it is directly connected to the host
        if src not in self.mac_to_switch_port:
            self.mac_to_switch_port[src] = (dpid, in_port)

        # Fall back on broadcasting by STP algorithm if destination is unknown
        if dst not in self.mac_to_switch_port:
            return self.broadcast_stp(ev)

        # If destination is known, then build a route with genetic algorithm
        self.pathfinder(ev)

    def broadcast_stp(self, ev):
        """ Send packet on all ports defined in spanning tree (to avoid loops like with default flooding) """
        msg = ev.msg
        dp = msg.datapath
        ofp = dp.ofproto
        dpid = dp.id

        self.setup_spanning_tree(dpid)
        if self.spanning_tree is None:
            self.logger.warn("Cannot build spanning tree! Topology is not connected!")
            return

        stp_forbidden_ports = set()
        if dpid in self.spanning_tree:
            # Collect all allowed by stp ports of the switch
            stp_allowed_ports = {metrics['port'] for dst_node, metrics in self.spanning_tree[dpid].items()}
            # Collect all ports of link tree (stp allowed and forbidden)
            all_link_tree_ports = {metrics['port'] for dst_node, metrics in self.link_tree[dpid].items()}
            stp_forbidden_ports = all_link_tree_ports.difference(stp_allowed_ports)

        always_skip_ports = {msg.in_port, ofp.OFPP_LOCAL}
        # Get all ports (stp allowed + forbidden + hosts + local)
        all_ports = set(dp.ports.keys())

        # Remove all except hosts and stp allowed ports
        allowed_ports = all_ports.difference(stp_forbidden_ports).difference(always_skip_ports)
        # Send the packet to all allowed ports on the switch
        for port_id in allowed_ports:
            output_packet_port(msg, dp, port_id)

    def setup_spanning_tree(self, dpid):
        """ Get topology data, create link tree if needed, build spanning tree  """
        # If link_tree is None, then spanning_tree need an update
        if self.link_tree is None or self.spanning_tree is None:
            switches, links = self.get_topology_data()
            if len(switches) == 1:
                # If it is a single node, no topology links (except hosts)
                self.link_tree, self.spanning_tree = {}, {}
                return
            self.link_tree = build_link_tree(links)
            remove_cycles(self.link_tree)  # Delete node-to-same-node connections
            remove_identical_links(self.link_tree)  # Pick the best link among each two nodes
            self.spanning_tree = build_spanning_tree(self.link_tree, switches, dpid)
            if self.spanning_tree is not None:
                set_reverse_links(self.spanning_tree, self.link_tree)

    def get_topology_data(self):
        """ Retrieve switches and links of the topology """
        switch_list = get_switch(self, None)
        switches = [switch.dp.id for switch in switch_list]
        link_list = get_link(self, None)
        links = [  # todo: get length by api for ports
            (link.src.dpid, link.dst.dpid, {'port': link.src.port_no, 'length': 1}) for link in link_list
        ]
        # self.logger.info('switches: {}, links: {}'.format(switches, links))
        return switches, links

    def pathfinder(self, ev):
        """ Find path and install flow for packets with known destination """
        msg = ev.msg
        dp = msg.datapath
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        dst = eth.dst
        src = eth.src
        src_dpid = dp.id
        src_port = msg.in_port

        # Get the destination from table
        dst_dpid, dst_port = self.mac_to_switch_port[dst]

        # Find a path with installed ports: [(switch id, in port, out port), ...]
        forward_path = self.find_path(src_dpid, src_port, dst_dpid, dst_port)
        if forward_path is not None:
            self.install_path(forward_path, src, dst)
        else:
            self.logger.warn("Unable to find path! Topology is not connected!")

        # Build and install the reverse path for future response packets
        reverse_path = self.find_path(dst_dpid, dst_port, src_dpid, src_port)
        if reverse_path is not None:
            self.install_path(reverse_path, dst, src)
        else:
            self.logger.warn("Unable to find path! Topology is not connected!")

        # Send packet on dst_port of destination switch (following packets will use installed flow path)
        output_packet_port(msg, get_switch(self, dst_dpid)[0].dp, dst_port)

    def find_path(self, src_dpid, src_port, dst_dpid, dst_port):
        """ Using routing algorithm find a path from src to dst and install ports to it """
        self.logger.info(f"Finding path from dp {src_dpid} to dp {dst_dpid} on port {dst_port}")

        if src_dpid == dst_dpid:
            # if source and destination are the same switch
            return [(src_dpid, src_port, dst_port)]

        self.setup_topology_graph()

        # Setup parameters for GA
        population_size = round(len(self.switches) * 1.5)
        # Using genetic algorithm to find optimal path
        idx_path = genetic(
            self.topology_graph.data, self.switch_to_idx[src_dpid], self.switch_to_idx[dst_dpid],
            population_size=population_size,
        )

        # Using Dijkstra's algorithm for comparison
        # idx_path = dijkstra(self.topology_graph.data, self.switch_to_idx[src_dpid], self.switch_to_idx[dst_dpid])[0]

        # Convert indexed path back to dpids
        path = [self.switches[idx] for idx in idx_path]
        path = install_ports_to_path(path, src_port, dst_port, self.link_tree)
        return path

    def setup_topology_graph(self):
        """ Create graph, link_tree, switch_to_idx and switches if needed """
        if self.topology_graph is None or self.link_tree is None:
            self.switches, links = self.get_topology_data()
            self.link_tree = build_link_tree(links)
            remove_cycles(self.link_tree)  # Delete node-to-same-node connections
            remove_identical_links(self.link_tree)  # Pick the best link among each two nodes
            self.topology_graph = self.create_mapped_graph(self.switches, links)

    def create_mapped_graph(self, switches: list[int], links: list[tuple[int, int, dict]]) -> Graph:
        """ Map dpid`s to their indexes, then create a graph of the indexes to use it with my routing algorithms.
            Including genetic algorithm.  """
        num_nodes = len(switches)
        length = 1
        self.switch_to_idx = {switches[idx]: idx for idx in range(num_nodes)}
        edges = [(self.switch_to_idx[source], self.switch_to_idx[dest], length) for source, dest, metrics in links]
        graph = Graph(num_nodes, edges, is_directed=True)
        return graph

    def install_path(self, path, src, dst):
        """ Take ported path and install flows to every switch in the path from src_port to dst_port, for
            every packet from src to dst which comes on the specified in_port. """
        for dpid, in_port, out_port in path:
            sw = get_switch(self, dpid)[0]
            dp = sw.dp
            ofp = dp.ofproto
            ofp_parser = dp.ofproto_parser

            # Create match for a packet
            match = ofp_parser.OFPMatch(in_port=in_port, dl_dst=haddr_to_bin(dst), dl_src=haddr_to_bin(src))

            # Define actions, which will be applied for the packet, which is matched
            actions = [ofp_parser.OFPActionOutput(out_port)]

            # Create Flow
            mod = ofp_parser.OFPFlowMod(
                datapath=dp, match=match, actions=actions,
                idle_timeout=0, hard_timeout=0, cookie=0,
                priority=ofp.OFP_DEFAULT_PRIORITY, command=ofp.OFPFC_ADD,
                flags=ofp.OFPFF_SEND_FLOW_REM)

            # Send message to switch to install this Flow
            dp.send_msg(mod)

        self.logger.info("Successfully installed path!")
