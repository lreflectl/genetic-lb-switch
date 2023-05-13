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


class Hw9Switch(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_0.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(Hw9Switch, self).__init__(*args, **kwargs)
        self.spanning_tree = None
        # End-host MAC and Location learning.  This is necessary because the
        # MAC addresses of end-hosts aren't known until they send a packet.
        self.mac_to_switch_port = {}

    def output_packet_port(self, msg, dp, port):
        # Get handles
        ofp = dp.ofproto
        ofp_parser = dp.ofproto_parser

        # Create an OpenFlow Output Action message for the given port
        actions = [ofp_parser.OFPActionOutput(port)]
        out = ofp_parser.OFPPacketOut(
            datapath=dp, buffer_id=ofp.OFP_NO_BUFFER, in_port=msg.in_port,
            actions=actions, data=msg.data)
        dp.send_msg(out)

    def get_topology_data(self):
        switch_list = get_switch(self, None)
        # Useful for getting familiar with the data structures
        # self.logger.info(switch_list[0].to_dict())
        switches = [switch.dp.id for switch in switch_list]
        link_list = get_link(self, None)
        links = [(link.src.dpid, link.dst.dpid, {'port': link.src.port_no}) for link in link_list]

        # Uncomment to look at the topology
        self.logger.info('switches: {}, links: {}'.format(switches, links))

        # A Graph = (V, E) = (switch_list, link_list)
        return switch_list, link_list

    def build_spanning_tree(self):
        # Note: Getting the graph will be helpful for building and storing a
        # spanning tree and then broadcasting over it
        graph = self.get_topology_data()
        switch_list, edge_list = graph
        print('Graph: ', graph)

        #
        # HW9TODO: Build a spanning tree, and save it as some data structure
        # that you can use later in broadcast_stp
        #



        # Save the spanning tree for use with all future broadcasts
        # self.spanning_tree = None
        pass

    def broadcast_stp(self, ev):
        self.logger.info('Broadcast STP:')

        # Build the spanning tree if this is the first time getting here
        if self.spanning_tree is None:
            self.build_spanning_tree()

        # Get handles
        msg = ev.msg
        dp = msg.datapath
        ofp = dp.ofproto
        ofp_parser = dp.ofproto_parser

        # This will show you that the switch is connecting via OpenFlow v1.0.
        # See ryu/ryu/ofproto/ofproto_v1_0.py for more information.
        # self.logger.info(ofp, ofp.OFP_VERSION)

        # Get the ports that the packet should not be broadcast on.  This
        # includes the switch's local port, the input port, and any port that
        # is not in the spanning tree on the appropriate ports
        always_skip_ports = [msg.in_port, ofp.OFPP_LOCAL]
        #
        # HW9TODO: Add in the ports that are not in the spanning tree for this
        # switch
        #
        spanning_tree_skip_ports = []  # TODO
        skip_port_set = set(always_skip_ports + spanning_tree_skip_ports)

        # For every port not being skipped, send the packet out that port.
        # Note: it is crucially important to flood out the ports that an
        # end-host is connected to
        # Note: When the packet is buffered at the switch, buffer_id can only be
        # used once.  Because of this, we use the packet data in the PacketIn.
        # Note: However, it is not required for switches to include the entire
        # packet as data, so even this simple function is not guaranteed to be
        # correct. OFPC could be used to ensure correctness, but that is
        # outside the scope of this homework.

        for port_num, port in dp.ports.items():
            assert (port_num == port.port_no)  # Sanity checking
            if port_num not in skip_port_set:
                self.output_packet_port(msg, dp, port_num)

    def install_path(self, path, src, dst):
        # Log
        self.logger.info("Installing path")

        for dpid, in_port, out_port in path:
            sw = get_switch(self, dpid)[0]
            dp = sw.dp
            ofp = dp.ofproto
            ofp_parser = dp.ofproto_parser

            # Build the match for incoming packets
            match = ofp_parser.OFPMatch(in_port=in_port,
                                        dl_dst=haddr_to_bin(dst), dl_src=haddr_to_bin(src))

            # Build the actions (where to send the packet)
            actions = [ofp_parser.OFPActionOutput(out_port)]

            # Build the FlowMod
            mod = ofp_parser.OFPFlowMod(
                datapath=dp, match=match, actions=actions,
                cookie=0, idle_timeout=0, hard_timeout=0,
                priority=ofp.OFP_DEFAULT_PRIORITY, command=ofp.OFPFC_ADD,
                flags=ofp.OFPFF_SEND_FLOW_REM)

            # Send the FlowMod
            dp.send_msg(mod)

    def find_path(self, src_dpid, src_port, dst_dpid, dst_port):
        self.logger.info("Finding path {}.p{} -> {}.p{}".format(src_dpid, src_port,
                                                                dst_dpid, dst_port))

        #
        # HW9TODO: Write a general purpose algorithm to find a path in the graph.
        #

        # HW9 Note: As an example, the below code installs all "trivial" paths
        # where the source and destination share the same switch. You need to
        # write an algorithm that searches the topology to find a path.
        if src_dpid == dst_dpid:
            path = [(src_dpid, src_port, dst_port)]
        else:
            path = None

        return path

    def per_flow(self, ev):
        self.logger.info('Per-Flow:')

        # Get Handles
        msg = ev.msg
        dp = msg.datapath
        ofproto = dp.ofproto

        # Parse the packet to get the Ethernet dst and src
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            # Ignore LLDPPackets used for topology discovery
            return

        dst = eth.dst
        src = eth.src

        # Learn MAC Addresses because MAC addresses of end-hosts are unknown
        # until the end-host sends a packet
        src_dpid = dp.id
        src_port = msg.in_port
        self.mac_to_switch_port[src] = (src_dpid, src_port)

        # print("Mac to switch port:")
        # print(self.mac_to_switch_port)
        # {'00:00:00:00:00:01': (1, 1),
        #  'da:70:07:55:5c:17': (1, 2),
        #  '6e:df:3a:48:f7:1c': (2, 3),
        #  '6a:9a:d8:30:29:cc': (2, 2),
        #  '2e:f5:2a:14:95:d3': (3, 2),
        #  '00:00:00:00:00:02': (2, 1),
        #  '00:00:00:00:00:03': (3, 1)
        # }

        # DEBUG
        # self.logger.info("packet in %s %s %s %s", src_dpid, src, dst, msg.in_port)

        # Fall back on STP Broadcast if we have not seen the destination yet
        if dst not in self.mac_to_switch_port:
            # return self.broadcast_stp(ev)
            return

        # Find the destination
        dst_dpid, dst_port = self.mac_to_switch_port[dst]

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
        self.output_packet_port(msg, get_switch(self, dst_dpid)[0].dp, dst_port)

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def packet_in_handler(self, ev):
        # Routing algorithm
        self.per_flow(ev)
