def output_packet_port(msg, dp, port):
    """ Sends the packet directly to specified port of the switch """
    ofp = dp.ofproto
    ofp_parser = dp.ofproto_parser

    # Create an OpenFlow Output Action message for the given port
    actions = [ofp_parser.OFPActionOutput(port)]
    out = ofp_parser.OFPPacketOut(
        datapath=dp, buffer_id=ofp.OFP_NO_BUFFER, in_port=msg.in_port,
        actions=actions, data=msg.data)
    dp.send_msg(out)


def install_ports_to_path(path: list[int], src_port: int, dst_port: int, link_tree: dict[int, dict[int, dict]]) \
        -> list[tuple[int, int, int]] or None:
    """ Choose links with the best bandwidth and install corresponding ports to every switch in the path """
    ported_path = []

    if len(path) < 1:
        return None
    if len(path) < 2:
        return [(path[0], src_port, dst_port)]
    print('path =', path)
    # best_link = min(link_tree[path[0]][path[1]], key=lambda link: link['length']) # todo
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
