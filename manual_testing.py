import fnss
from mininet.net import Mininet
from mininet.link import TCLink
from mininet.node import RemoteController, Host
from mininet.log import setLogLevel
from mininet.cli import CLI


def main():
    fnss_topo = fnss.fat_tree_topology(k=4)

    fnss.set_delays_constant(fnss_topo, 2, 'ms')
    fnss.set_capacities_constant(fnss_topo, 100, 'Mbps')
    fnss.set_buffer_sizes_constant(fnss_topo, 50, 'packets')

    # Convert topo to Mininet and change nodes labels
    mininet_topo = fnss.to_mininet(fnss_topo, relabel_nodes=True)

    # Create and start mininet with remote controller to connect GLBSwitch
    net = Mininet(topo=mininet_topo, link=TCLink, controller=RemoteController)
    net.start()
    CLI(net)
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    main()
