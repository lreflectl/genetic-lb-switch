import matplotlib.pyplot as plt
from matplotlib import pyplot
import csv

timestamps = []
bws = []
jitters = []
latencies = []
with open('genetic_results.csv') as csvfile:
    reader = csv.reader(csvfile)
    reader.__next__()  # skip headers
    for line in reader:
        timestamps.append(float(line[0]))
        bws.append(float(line[1]))
        jitters.append(float(line[2]))
        latencies.append(float(line[3]))


dijkstra_timestamps = []
dijkstra_bws = []
dijkstra_jitters = []
dijkstra_latencies = []
with open('dijkstra_results.csv') as csvfile:
    reader = csv.reader(csvfile)
    reader.__next__()  # skip headers
    for line in reader:
        dijkstra_timestamps.append(float(line[0]))
        dijkstra_bws.append(float(line[1]))
        dijkstra_jitters.append(float(line[2]))
        dijkstra_latencies.append(float(line[3]))


stp_timestamps = []
stp_bws = []
stp_jitters = []
stp_latencies = []
with open('stp_results.csv') as csvfile:
    reader = csv.reader(csvfile)
    reader.__next__()  # skip headers
    for line in reader:
        stp_timestamps.append(float(line[0]))
        stp_bws.append(float(line[1]))
        stp_jitters.append(float(line[2]))
        stp_latencies.append(float(line[3]))

fig1, ax = plt.subplots(figsize=(10, 4))
ax.plot(timestamps, bws, 'b', label='Генетичний алгоритм')
ax.plot(dijkstra_timestamps, dijkstra_bws, 'g', label='Алгоритм Дійкстри')
ax.plot(stp_timestamps, stp_bws, 'r', label='Без балансування')
ax.set_ylabel('Пропускна здатність мережі (Mbit/s)')
ax.set_xlabel('Час (sec)')
ax.legend()
plt.savefig('./plots/bw_mbit.png')
plt.show()

fig2, ax = plt.subplots(figsize=(10, 4))
ax.plot(timestamps, jitters, 'b', label='Генетичний алгоритм')
ax.plot(dijkstra_timestamps, dijkstra_jitters, 'g', label='Алгоритм Дійкстри')
ax.plot(stp_timestamps, stp_jitters, 'r', label='Без балансування')
ax.set_ylabel('Джиттер в мережі (ms)')
ax.set_xlabel('Час (sec)')
ax.legend()
plt.savefig('./plots/jitter_ms.png')
plt.show()

fig3, ax = plt.subplots(figsize=(10, 4))
ax.plot(timestamps, latencies, 'b', label='Генетичний алгоритм')
ax.plot(dijkstra_timestamps, dijkstra_latencies, 'g', label='Алгоритм Дійкстри')
ax.plot(stp_timestamps, stp_latencies, 'r', label='Без балансування')
ax.set_ylabel('Затримка мережі (ms)')
ax.set_xlabel('Час (sec)')
ax.legend()
plt.savefig('./plots/latency_ms.png')
plt.show()
