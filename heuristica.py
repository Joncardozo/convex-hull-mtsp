import random
import math
import matplotlib.pyplot as plt
import itertools

# classe do nó
class Node:
    def __init__(self, x, y, index):
        self.x = x
        self.y = y
        self.index = index

    def distance(self, node):
        """calcula distancia euclidiana (nao da forma tsplib)."""
        return math.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)


def calculate_total_cost(tours, depot):
    """calcula distancia total viajada."""
    total_cost = 0
    for tour in tours:
        if not tour:
            continue
        current_tour_nodes = [depot] + tour + [depot]
        for i in range(len(current_tour_nodes) - 1):
            total_cost += current_tour_nodes[i].distance(current_tour_nodes[i+1])
    return total_cost

def calculate_partial_cost(tour, depot):
    """calcula a distancia total de um caixeiro."""
    cost = 0
    # print(tour)
    tour_nodes = [depot] + tour + [depot]
    for i in range(len(tour_nodes) - 1):
        cost += tour_nodes[i].distance(tour_nodes[i+1])
    return cost

def generate_random_nodes(num_nodes):
    """nós aleatorios."""
    nodes = []
    i = 0
    for _ in range(num_nodes -1):
        nodes.append(Node(random.uniform(0, 100), random.uniform(0, 100), i))
        i += 1
    return nodes

def convex_hull(nodes):
    """
    convex hull -> algoritmo graham scan
    """
    if not nodes:
        return []
    # no com a mais baixa coordenada y
    start_node = min(nodes, key=lambda node: (node.y, node.x))

    # ordena com os angulos
    sorted_nodes = sorted(nodes, key=lambda node: (math.atan2(node.y - start_node.y, node.x - start_node.x), node.distance(start_node)))

    hull = []
    for node in sorted_nodes:
        # se o angulo dos ultios dois nos nao for anti horario, remove o do meio
        while len(hull) >= 2 and cross_product(hull[-2], hull[-1], node) <= 0:
            hull.pop()
        hull.append(node)

    return hull

def cross_product(p1, p2, p3):
    """produto externo para a definicao da orientacao."""
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)

def plot_tours(tours, all_nodes, title):
    """plota mtsp."""
    plt.figure(figsize=(10, 10))
    colors = ['r', 'g', 'c', 'm', 'y', 'b', 'orange', 'purple']
    depot = all_nodes[0]

    # plota todos os nos
    for node in all_nodes[1:]:
        plt.plot(node.x, node.y, 'ko', markersize=5)

    # garagem
    plt.plot(depot.x, depot.y, 'k*', markersize=20, label='Depot')

    # tours
    for i, tour in enumerate(tours):
        if not tour:
            continue
        color = colors[i % len(colors)]
        # full tour com depot no inicio e final
        full_tour = [depot] + tour
        for j in range(len(full_tour)):
            start_node = full_tour[j]
            # conecta o final ao inicio
            end_node = full_tour[(j + 1) % len(full_tour)]
            plt.plot([start_node.x, end_node.x], [start_node.y, end_node.y], color=color, linestyle='-', marker='o', markersize=4, label=f'Salesman {i+1}' if j == 0 else "")

    # legenda
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    plt.title(title)
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.show()

def get_salesman_location_at_time(tour, depot, time_t):
    """
    Determines a salesman's location at a specific time t.
    Assumes speed is 1 distance unit per time unit.
    """
    full_tour = [depot] + tour
    cumulative_time = 0

    for i in range(len(full_tour)):
        p_start = full_tour[i]
        # The next stop is either the next node in the tour or back to the depot
        p_end = full_tour[i + 1] if i + 1 < len(full_tour) else depot

        segment_duration = p_start.distance(p_end)

        if cumulative_time + segment_duration >= time_t:
            # The salesman is on this segment at time_t
            time_into_segment = time_t - cumulative_time
            # Avoid division by zero for zero-length segments (staying at a node)
            progress = time_into_segment / segment_duration if segment_duration > 0 else 0

            x = p_start.x + progress * (p_end.x - p_start.x)
            y = p_start.y + progress * (p_end.y - p_start.y)
            return Node(x, y, 9999)

        cumulative_time += segment_duration

    # If time_t is beyond the tour's completion, the salesman is back at the depot
    return depot

def calculate_max_separation_at_arrivals(tours, depot):
    """
    At each arrival event, calculates the maximum distance between any pair of salesmen.
    """
    arrival_events = []
    # 1. Generate all arrival events for all salesmen
    for salesman_idx, tour in enumerate(tours):
        if not tour:
            continue

        full_tour = [depot] + tour
        cumulative_time = 0
        for i in range(len(full_tour)):
            p_start = full_tour[i]
            p_end = full_tour[i+1] if i+1 < len(full_tour) else depot

            segment_duration = p_start.distance(p_end)
            arrival_time = cumulative_time + segment_duration

            # The arrival is at p_end. We only need the time.
            arrival_events.append({
                "time": arrival_time,
                "salesman_idx": salesman_idx
            })
            cumulative_time = arrival_time

    # 2. Sort events chronologically and remove duplicate times
    arrival_events.sort(key=lambda x: x['time'])
    unique_arrival_times = sorted(list(set(event['time'] for event in arrival_events)))

    max_separations = []

    # 3. Process each unique arrival time
    for arrival_time in unique_arrival_times:
        # Get the location of every salesman at this specific time
        salesman_locations = [get_salesman_location_at_time(tour, depot, arrival_time) for tour in tours]

        current_max_dist = 0

        # Find the maximum distance between any pair of salesmen
        if len(salesman_locations) > 1:
            for loc1, loc2 in itertools.combinations(salesman_locations, 2):
                dist = loc1.distance(loc2)
                if dist > current_max_dist:
                    current_max_dist = dist

        if current_max_dist > 0:
             max_separations.append({
                "time": arrival_time,
                "max_dist": current_max_dist
            })

    return max_separations


def main():
    """funcao de entrada."""
    num_nodes = 100
    num_salesmen = 3

    nodes = generate_random_nodes(num_nodes)
    depot = nodes[0]

    # todos comecam na garagme
    unassigned_nodes = nodes[1:]

    # tour de cada caixeiro
    salesmen_tours = [[] for _ in range(num_salesmen)]

    # convex hull inicial para cada salesman
    for i in range(num_salesmen):
        # no minimo 3 nos sobrando para fazer um hull
        if len(unassigned_nodes) < 3:
            break

        hull = convex_hull(unassigned_nodes)
        salesmen_tours[i] = hull

        # tira o no assinalado da lista de nao assinalados
        unassigned_nodes = [node for node in unassigned_nodes if node not in hull]

    # cheapest insertion
    while unassigned_nodes:
        best_insertion_cost = float('inf')
        best_node_to_insert = None
        best_tour_idx = -1
        best_insertion_index = -1

        # nó de menor custo a ser adicionado para todos os caixeiros
        for node_to_insert in unassigned_nodes:
            for tour_idx, tour in enumerate(salesmen_tours):
                # tour vazio
                if not tour:
                    cost = depot.distance(node_to_insert) * 2
                    if cost < best_insertion_cost:
                        best_insertion_cost = cost
                        best_node_to_insert = node_to_insert
                        best_tour_idx = tour_idx
                        best_insertion_index = 0 # It will be the first node in this tour
                    continue

                # tour ja iniciado: acha aresta de menor valor
                full_tour = [depot] + tour
                for i in range(len(full_tour)):
                    p1 = full_tour[i]
                    p2 = full_tour[(i + 1) % len(full_tour)]

                    cost = p1.distance(node_to_insert) + p2.distance(node_to_insert) - p1.distance(p2)

                    if cost < best_insertion_cost:
                        best_insertion_cost = cost
                        best_node_to_insert = node_to_insert
                        best_tour_idx = tour_idx
                        # indice de insercao dentro do tour
                        best_insertion_index = i

        if best_node_to_insert:
            # melhor no, melhor posicao, melhor tour
            salesmen_tours[best_tour_idx].insert(best_insertion_index, best_node_to_insert)
            unassigned_nodes.remove(best_node_to_insert)
        else:
            # se nos nao assinalados esta vazio
            break


    # imprime tours
    for i,j in enumerate(salesmen_tours):
        tour_order = [x.index for x in j]
        partial_cost = calculate_partial_cost(j, nodes[0])
        print(f"tour {i}: ", tour_order, f"custo: {partial_cost}")

    # imprime custo total
    total_cost = calculate_total_cost(salesmen_tours, nodes[0])
    print(f"custo_total: {total_cost}")

    max_separation = calculate_max_separation_at_arrivals(salesmen_tours, nodes[0])

    critical_event = max_separation[0]
    for e in max_separation:
        if e['max_dist'] > critical_event['max_dist']:
            critical_event = e

    print(f"distancia maxima: {critical_event}")



    plot_tours(salesmen_tours, nodes, f"mTSP solucao para {num_salesmen} caixeiros-viajantes")



if __name__ == "__main__":
    main()
