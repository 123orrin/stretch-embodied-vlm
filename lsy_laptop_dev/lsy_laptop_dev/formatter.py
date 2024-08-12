moves = 'list_1 = [forward, backward] | list_2 = [0.02, 0.05]'
moves = moves.replace('list_1 = [', '')
moves = moves.replace(' list_2 = [', '')
moves = moves.replace(']', '')
moves = moves.replace('"', '').replace(' ', '')
moves = moves.split('|')
print(moves)
directions = moves[0]
distances = moves[1]
print(directions)
print(distances)
directions = directions.split(',')
distances = distances.split(',')	
print(directions)
print(distances)