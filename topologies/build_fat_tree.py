import fnss

# Create fat-tree topology with 4 pods
topology = fnss.fat_tree_topology(k=4)

fnss.set_capacities_constant(topology, 100, "Mbps")
fnss.set_delays_constant(topology, 10, "ns")

fnss.write_topology(topology, 'fat_tree_4pods_100Mbps_10ns.xml')
