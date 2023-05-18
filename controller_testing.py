import fnss
from mininet.net import Mininet
from mininet.link import TCLink
from mininet.node import RemoteController, Host
from mininet.log import setLogLevel, lg
import time


def measure_jitter(h1: Host, h2: Host):
    h2.sendCmd('iperf -s -u')
    jitter_msg = h1.cmd(f'iperf -u -c {h2.IP()}')
    h2.sendInt()
    h2.waitOutput()
    jitter_ms = float(jitter_msg.splitlines()[-1].split(' ')[-4])
    lg.info('Iperf udp results: \n')
    lg.info(jitter_msg)
    lg.info(f"{jitter_ms} ms")
    lg.info('\n----\n')

    return jitter_ms


def main():
    # Create Fat-Tree topo with 4 pods (16 hosts)
    fnss_topo = fnss.fat_tree_topology(k=4)  # 2 pods while test

    fnss.set_delays_constant(fnss_topo, 2, 'ms')
    fnss.set_capacities_constant(fnss_topo, 100, 'Mbps')
    # link_types = nx.get_edge_attributes(fnss_topo, 'type')
    # core_agg_links = [link for link in link_types
    #                   if link_types[link] == 'core_aggregation']
    # fnss.set_capacities_random_uniform(fnss_topo, list(range(90, 100)), 'Mbps')

    fnss.set_buffer_sizes_constant(fnss_topo, 50, 'packets')

    # Convert topo to Mininet and change nodes labels
    mininet_topo = fnss.to_mininet(fnss_topo, relabel_nodes=True)

    # Create and start mininet with remote controller to connect GLBSwitch
    net = Mininet(topo=mininet_topo, link=TCLink, controller=RemoteController)
    net.start()

    # Wait for the controller
    input('Press any key when controller is ready')

    h1, h16 = net.hosts[0], net.hosts[15]
    server_hosts = net.hosts[8:15]
    client_hosts = net.hosts[1:8]
    for server_host in server_hosts:
        server_host.sendCmd('iperf -s -u')
        time.sleep(0.2)
    server_id = 9
    for client_host in client_hosts:
        client_host.sendCmd(f'iperf -c 10.0.0.{server_id} -u -t 9999 -b 50M -d -y C')
        server_id += 1
        time.sleep(0.2)

    lg.info('Traffic generation started\n')
    input('Press any key to proceed')

    with open('results/results.csv', 'w') as file:
        start_time = time.time()
        file.write('time, bw_mbits, jitter_ms, latency_ms\n')
        for i in range(40):
            measure_time = time.time() - start_time
            file.write(str(measure_time) + ',')
            # ----------- Measure Bandwidth ----------
            result_bws = net.iperf((h1, h16), seconds=3)
            server_bw = float(result_bws[0].split(' ')[0])
            client_bw = float(result_bws[1].split(' ')[0])
            bw_mbits = min(server_bw, client_bw)
            file.write(str(bw_mbits) + ',')
            lg.info('Iperf tcp results: ')
            lg.info(bw_mbits)
            lg.info('\n----\n')
            # ----------- Measure Jitter ------------
            jitter_ms = measure_jitter(h1, h16)
            file.write(str(jitter_ms) + ',')
            # ----------- Measure Latency -----------
            ping_results = net.pingFull((h1, h16))
            avg_ping = ping_results[0][2][3]
            file.write(str(avg_ping) + '\n')
            lg.info('Ping results: ')
            lg.info(avg_ping)
            lg.info('\n----\n')

            # time.sleep(5)

    # Stop traffic generation
    for server_host in server_hosts:
        server_host.sendInt()
        server_host.waitOutput()
    for client_host in client_hosts:
        client_host.sendInt()
        output = client_host.waitOutput()
        lg.info("Traffic flood: " + output)
    lg.info("Traffic generation stopped.\n")
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    main()
