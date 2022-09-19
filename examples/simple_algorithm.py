import numpy as np

def find_trace(pos1, pos2):
    py1, px1 = pos1
    py2, px2 = pos2

    if (px1 == px2):
        # vertical line
        result = []
        for i in range(py1, py2+1):
            result.append((px1, i))
        return result
    
    if (py1 == py2):
        # horizontal line
        result = []
        for i in range(px1, px2+1):
            result.append((i, py1))

        return result

    slop = (py1 - py2) / (px1-px2)

    c = py1 - slop * px1

    start = min(px1, px2)
    end = max(px1, px2) + 1

    result = []

    for i in range(start, end):
        newx = i
        newy = int(slop * newx + c)
        result.append((newx, newy))

    return result

def fill_in_line(arr, pos1, pos2):
    trace_pos = find_trace(pos1, pos2)
    for pos in trace_pos:
        arr[pos[1]][pos[0]] = 1    
    
