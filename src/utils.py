def _idx_dec(row, col):
    return row + col * n


def _idx_enc(index):
    """
    :param index:
    :return row, col:
    """
    return index % n, index // n


n = int(input())
start_point = 0
maze_size = n * n
graph = [[[] for _ in range(n)] for _ in range(n)]
for j in range(n):
    for i in range(n):
        index = _idx_dec(i, j)
        adj_list = list(map(int, input().split()))
        for element in adj_list:
            graph[i][j].append(element)
exit_point = int(input())

