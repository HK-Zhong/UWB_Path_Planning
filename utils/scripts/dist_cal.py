import math

def input_2d_distance():
    x1, y1 = map(float, input("请输入第一个点坐标 x1 y1: ").split())
    x2, y2 = map(float, input("请输入第二个点坐标 x2 y2: ").split())

    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    print(f"两点之间的距离为: {dist}")


if __name__ == '__main__':

    # 调用
    while True:
        input_2d_distance()