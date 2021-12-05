if __name__ == '__main__':
    EPS = 0.00000000000001
    f = open("values.txt", "r")
    values = []
    for line in f.readlines():
        if (len(line) > 4):
            l = line.split()
            values.append(float(l[3]))
    f.close()
    length = len(values)//2
    first = values[:length]
    second = values[length:]
    not_eq_vals = []
    for i in range(length):
        if (abs(first[i] - second[i]) > EPS):
            not_eq_vals.append((i, first[i], second[i]))
    if (len(not_eq_vals) == 0):
        print("All values are equal")
    else:
        for item in not_eq_vals:
            print(f"Values in position {item[0]} are not equal: {item[1]} and {item[2]}")