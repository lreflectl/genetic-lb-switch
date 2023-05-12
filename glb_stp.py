from ryu.controller.handler import MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.lib import stplib
from ryu.lib.packet import packet
from ryu.lib.packet import ether_types
from ryu.lib.packet import ethernet
from ryu.app import simple_switch_stp_13
from ryu.controller.ofp_event import EventOFPMsgBase
from ryu.topology.api import get_switch, get_link
from ryu.lib import dpid as dpid_lib
from ryu.topology.switches import Link

import sys
import os
sys.path.append(os.path.dirname(os.path.expanduser('~/py/genetic-pathfinder/baseline_algorithms.py')))
sys.path.append(os.path.dirname(os.path.expanduser('~/py/genetic-pathfinder/python_graph.py')))
from baseline_algorithms import dijkstra
from python_graph import Graph

# from genetic_pathfinder.python_graph import Graph
# from genetic_pathfinder.baseline_algorithms import dijkstra


def output_packet_port(msg, dp, port):
    print('Sent packet directly to dst port')
    # Get handles
    ofp = dp.ofproto
    ofp_parser = dp.ofproto_parser

    # Create an OpenFlow Output Action message for the given port
    actions = [ofp_parser.OFPActionOutput(port)]
    out = ofp_parser.OFPPacketOut(
        datapath=dp, buffer_id=ofp.OFP_NO_BUFFER, in_port=msg.match['in_port'],
        actions=actions, data=msg.data)
    dp.send_msg(out)


def build_link_tree(links: list[tuple[int, int, dict]]):
    """ Create hash map of the links and their metrics """
    link_tree = {}
    for link in links:
        link_tree.setdefault(link[0], {})
        link_tree[link[0]].setdefault(link[1], [])
        link_tree[link[0]][link[1]].append(link[2])
    return link_tree


def install_ports_to_path(path: list[int], src_port: int, dst_port: int, link_tree) \
        -> list[tuple[int, int, int]] or None:
    """ Choose links with the best bandwidth and install corresponding ports to every switch in the path """
    ported_path = []

    if len(path) < 1:
        return None
    if len(path) < 2:
        return [(path[0], src_port, dst_port)]
    print('path =', path)
    first_sw = (path[0], src_port, link_tree[path[0]][path[1]][0]['port'])
    ported_path.append(first_sw)
    for idx in range(1, len(path) - 1):
        prev_sw = path[idx-1]
        curr_sw = path[idx]
        next_sw = path[idx+1]
        ported_sw = (curr_sw, link_tree[curr_sw][prev_sw][0]['port'], link_tree[curr_sw][next_sw][0]['port'])
        ported_path.append(ported_sw)
    last_sw = (path[-1], link_tree[path[-1]][path[-2]][0]['port'], dst_port)
    ported_path.append(last_sw)

    return ported_path


class GeneticSwitch(simple_switch_stp_13.SimpleSwitch13):
    def __init__(self, *args, **kwargs):
        super(GeneticSwitch, self).__init__(*args, **kwargs)
        # End-host MAC and Location learning.  This is necessary because the
        # MAC addresses of end-hosts aren't known until they send a packet.
        self.mac_to_switch_port = {}
        self.switch_to_idx = {}
        self.graph = Graph

    def install_path(self, path, src, dst):
        self.logger.info("Installing path")

        for dpid, in_port, out_port in path:
            sw = get_switch(self, dpid)[0]
            dp = sw.dp
            ofp = dp.ofproto
            ofp_parser = dp.ofproto_parser

            # Build the match
            match = ofp_parser.OFPMatch(in_port=in_port, eth_dst=dst, eth_src=src)

            # Build the actions
            actions = [ofp_parser.OFPActionOutput(out_port)]
            inst = [ofp_parser.OFPInstructionActions(ofp.OFPIT_APPLY_ACTIONS,
                                                     actions)]

            # Build the FlowMod
            mod = ofp_parser.OFPFlowMod(
                datapath=dp, match=match, cookie=0,
                command=ofp.OFPFC_ADD, idle_timeout=0, hard_timeout=0,
                priority=ofp.OFP_DEFAULT_PRIORITY,
                flags=ofp.OFPFF_SEND_FLOW_REM, instructions=inst)

            # Send the FlowMod
            dp.send_msg(mod)

    def get_topology_data(self):
        switch_list = get_switch(self, None)
        switches = [switch.dp.id for switch in switch_list]
        link_list = get_link(self, None)
        links = [(link.src.dpid, link.dst.dpid, {'port': link.src.port_no}) for link in link_list]
        # Uncomment to look at the topology
        self.logger.info('switches: {}, links: {}'.format(switches, links))

        # A Graph = (V, E) = (switch_list, link_list)
        return switches, links

    def create_mapped_graph(self, switches: list[int], links: list[tuple[int, int, dict]]) -> Graph:
        num_nodes = len(switches)
        weight = 1
        self.switch_to_idx = {switches[idx]: idx for idx in range(num_nodes)}
        edges = [(self.switch_to_idx[source], self.switch_to_idx[dest], weight) for source, dest, _ in links]
        graph = Graph(num_nodes, edges, is_directed=True)
        return graph

    def find_path(self, src_dpid, src_port, dst_dpid, dst_port):
        self.logger.info("Finding path {}.p{} -> {}.p{}".format(src_dpid, src_port,
                                                                dst_dpid, dst_port))
        # if source and destination are the same switch
        if src_dpid == dst_dpid:
            return [(src_dpid, src_port, dst_port)]

        switches, links = self.get_topology_data()
        self.graph = self.create_mapped_graph(switches, links)
        # print(self.switch_to_idx)
        # print(self.graph)
        link_tree = build_link_tree(links)
        print(link_tree)

        idx_path = dijkstra(self.graph.data, self.switch_to_idx[src_dpid], self.switch_to_idx[dst_dpid])[0]
        path = [switches[idx] for idx in idx_path]
        path = install_ports_to_path(path, src_port, dst_port, link_tree)
        print('ported path =', path)

        return path

    def per_flow(self, ev):
        # Get Handles
        msg = ev.msg
        dp = msg.datapath
        # Parse the packet to get the Ethernet dst and src
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]
        dst = eth.dst
        src = eth.src
        src_dpid = dp.id
        src_port = msg.match['in_port']

        # Find the destination
        dst_dpid, dst_port = self.mac_to_switch_port[dst]

        # self.logger.info("Per-flow: packet in %s.p%s -> %s.p%s (from %s to %s)",
        #                  src_dpid, src_port, dst_dpid, dst_port, src, dst)

        # Build and install the forward path
        # Path -> [(SW DPID, Input Port, Output Port), ...]
        fwd_path = self.find_path(src_dpid, src_port, dst_dpid, dst_port)
        if fwd_path is not None:
            self.install_path(fwd_path, src, dst)
        else:
            self.logger.warn("Unable to find path! Is the topology connected?")

        # Build and install the reverse path as well to avoid triggering
        # extra PacketIn messages
        rev_path = self.find_path(dst_dpid, dst_port, src_dpid, src_port)
        if rev_path is not None:
            self.install_path(rev_path, dst, src)
        else:
            self.logger.warn("Unable to find path! Is the topology connected?")

        # Output the packet directly to the destination because it will not use
        # the newly installed rule
        output_packet_port(msg, get_switch(self, dst_dpid)[0].dp, dst_port)

    @set_ev_cls(stplib.EventPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev: EventOFPMsgBase):
        pkt = packet.Packet(ev.msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        # Ignore LLDPPackets used for topology discovery
        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        dst = eth.dst
        src = eth.src
        dpid = ev.msg.datapath.id
        in_port = ev.msg.match['in_port']

        # learn a mac address for routing algorithm
        if src not in self.mac_to_switch_port:
            # save only the switch which is connected
            # directly to the host
            self.mac_to_switch_port[src] = (dpid, in_port)

        self.logger.info(f"Packet in port {in_port} of switch {dpid} ({src} to {dst})")

        # Fall back on default ryu STP Broadcast if we have not seen the destination yet
        if dst not in self.mac_to_switch_port:
            return self.broadcast_stp(ev)
            # self.logger.info('Mac of the dst has not been learned yet, skipping.')
            # return

        self.per_flow(ev)

    def broadcast_stp(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']

        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]

        dst = eth.dst
        src = eth.src

        dpid = datapath.id
        self.mac_to_port.setdefault(dpid, {})

        self.logger.info("STP: packet in port %s of switch %s, from %s to %s", in_port, dpid, src, dst)

        # learn a mac address to avoid FLOOD next time.
        self.mac_to_port[dpid][src] = in_port

        if dst in self.mac_to_port[dpid]:
            out_port = self.mac_to_port[dpid][dst]
        else:
            out_port = ofproto.OFPP_FLOOD

        actions = [parser.OFPActionOutput(out_port)]

        # install a flow to avoid packet_in next time
        if out_port != ofproto.OFPP_FLOOD:
            match = parser.OFPMatch(in_port=in_port, eth_dst=dst)
            self.add_flow(datapath, 1, match, actions)

        data = None
        if msg.buffer_id == ofproto.OFP_NO_BUFFER:
            data = msg.data

        out = parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id,
                                  in_port=in_port, actions=actions, data=data)
        datapath.send_msg(out)

    @set_ev_cls(stplib.EventTopologyChange, MAIN_DISPATCHER)
    def _topology_change_handler(self, ev):
        # Flush mac table for STP
        super(GeneticSwitch, self)._topology_change_handler(ev)
        # Clear mac table for routing algorithm as well
        new_mac_to_switch = {
            mac: switch_port for mac, switch_port in self.mac_to_switch_port.items()
            if switch_port[0] != ev.dp.id
        }
        self.mac_to_switch_port = new_mac_to_switch
