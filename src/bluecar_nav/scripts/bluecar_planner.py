import rospy
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import json

treasure_pos = []

#最基本的A*搜索算法
def A_star(map, start, end):
    """
    使用A*算法寻找起点到终点的最短路径
    :param map: 二维列表，表示地图。0表示可以通过的点，1表示障碍物。
    :param start: 元组，表示起点坐标。
    :param end: 元组，表示终点坐标。
    :return: 列表，表示从起点到终点的最短路径，其中每个元素是一个坐标元组。
    """
    # 定义启发式函数（曼哈顿距离）
    def heuristic(node1, node2):
        return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

    # 初始化open_list、closed_list、g_score、came_from
    open_list = [(0, start)]
    closed_list = set()
    g_score = {start: 0}
    came_from = {}

    # 开始搜索
    while open_list:
        # 取出f值最小的节点
        current = heapq.heappop(open_list)[1]
        if current == end:
            # 找到终点，返回路径
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        # 将当前节点加入closed_list
        closed_list.add(current)

        # 遍历相邻节点
        for neighbor in [(current[0] - 1, current[1]),
                         (current[0] + 1, current[1]),
                         (current[0], current[1] - 1),
                         (current[0], current[1] + 1)]:
            if 0 <= neighbor[0] < len(map) and 0 <= neighbor[1] < len(map[0]) and map[neighbor[0]][neighbor[1]] == 0:
                # 相邻节点是可通过的节点
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 如果相邻节点不在g_score中，或者新的g值更优，则更新g_score和came_from
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = current

    # 没有找到可行路径，返回空列表
    return []

#坐标变换函数，将10*10的坐标映射到地图矩阵上，方便用来可视化
def pose2map(x,y):
    return 21-2*y,x*2-1

def map2pose(x,y):
    return (21-y)/2,(x+1)/2

#地图上两点之间的最短路径
def A_star_length(map1,start_x,start_y,end_x,end_y):
    # 定义起点和终点
    start = (start_x, start_y)
    end = (end_x, end_y)

    # 计算最短路径
    start=pose2map(*start)
    end=pose2map(*end)
    path = A_star(map1, start, end)
    path_length=int((len(path)-1)/2)
    return path_length

#预计算
def precomputation(map1,start,end,mid_points):
    permutations_list = list(permutations(mid_points,2))
    length_dict={}
    length_dict[start]={}
    for pt1 in mid_points:
        length_dict[pt1]={}
        length_dict[start][pt1]=A_star_length(map1,start[0],start[1],pt1[0],pt1[1])
    for pt1 in mid_points:
        length_dict[pt1][end]=A_star_length(map1,pt1[0],pt1[1],end[0],end[1])
    for pt1,pt2 in permutations_list:
        length_dict[pt1][pt2]=A_star_length(map1,pt1[0],pt1[1],pt2[0],pt2[1])
    length_dict[start][end]=A_star_length(map1,start[0],start[1],end[0],end[1])
    return length_dict

#计算最短距离的路线
def get_min_path(map1,start,end,mid_points,length_dict=None):
    #穷举法8！=40320
    #计算1000个路径需要3s，全部计算需要2分钟计算太慢,但是使用路径查询后大大减少了计算量40320组数据在0.2s完成计算获得最优路径
    permutations_list = list(permutations(mid_points))
    min_path_length=float("inf")
    min_path=None
    for mid_points in permutations_list:
        mid_points=list(mid_points)
        mid_points.append(end)
        mid_points.insert(0,start)

        all_length=0
        for i in range(len(mid_points)-1):
            if length_dict:#如果没有预计算则采用现场计算，很费时
                length=length_dict[mid_points[i]][mid_points[i+1]]
            else:
                length=A_star_length(map1,mid_points[i][0],mid_points[i][1],mid_points[i+1][0],mid_points[i+1][1])
            all_length+=length
        if all_length<min_path_length:
            min_path_length=all_length
            min_path=mid_points

    return min_path,min_path_length 

#将10*10pose坐标映射到21*21的地图坐标上
def gennerate_all_path(map1,min_path):
    path=[]
    for i in range(len(min_path)-1):
        #start=pose2map(*start)
        #end=pose2map(*end)
        base_path=A_star(map1,pose2map(*min_path[i]),pose2map(*min_path[i+1]))
        path+=base_path[1:]
    path.insert(0,pose2map(*min_path[0]))
    return path

def multi_goal_Astar(map1,start,end,mid_points):
    '''
    含有中间位置的最短路径规划算法
    '''
    yujisuan=precomputation(map1,start,end,mid_points)
    min_path,min_path_length =get_min_path(map1,start,end,mid_points,yujisuan)

    #print(real) 
    
    return min_path,min_path_length

def multi_Astar(map1,start,end,mid_points):
    min_path,min_path_length=multi_goal_Astar(map1,start,end,mid_points)
    all_points=[]
    for i in range(len(min_path)-1):
        temp=A_star(map1,pose2map(*min_path[i]),pose2map(*min_path[i+1]))[:-1]
        for j in temp:
            all_points.append(j)
    all_points.append(pose2map(*min_path[-1]))
    real=[]
    for point in all_points:
        real.append(map2pose(point[0],point[1]))
    #print(all_points)
    real=real[::-1]
    return real
    

def find_turning_points(path):
    turning_points = []
    for i in range(1, len(path)-1):
        current = path[i]
        previous = path[i-1]
        next = path[i+1]
        if ((current[0]-previous[0]) * (next[1]-current[1]) != (current[1]-previous[1]) * (next[0]-current[0])) or(current[0]-previous[0]) * (next[0]-current[0]) + (current[1]-previous[1]) * (next[1]-current[1]) <0:#or (current[0]-previous[0]) * (next[1]-current[1])==(current[1]-previous[1]) * (next[0]-current[0]) ==0#(3,1) (3,3) (3,1) (3-3)*(1-3) (3-1)*(1-3)
            turning_points.append(current)


    return turning_points

def turn_direction(v1, pt1,pt2):
    x1,y1=pt1
    x2,y2=pt2
    # 计算下一个方向向量
    v2 = np.array([x2, y2]) - np.array([x1, y1])
    # 计算叉积
    cross = np.cross(v1, v2)
    dot_product = v1[0]*v2[0] + v1[1]*v2[1]
    norm_product = ((v1[0]**2 + v1[1]**2) *(v2[0]**2 + v2[1]**2))**0.5
    if cross > 0:
        return 'left'
    elif cross < 0:
        return 'right'
    elif dot_product==norm_product:
        return "straight"
    else:
        return 'Reverse direction'

def Magic(pos):
    start = (1,1)
    end = (10,10)
    map1=[
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
[1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1],
[1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1],
[1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1],
[1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,0,1,1,1,0,1],
[1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1],
[1,1,1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,1],
[1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
[1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,1,1,0,1],
[1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1],
[1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1],
[1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,1],
[1,1,1,0,1,1,1,0,1,1,1,1,1,0,1,0,1,0,1,1,1],
[1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1],
[1,0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1],
[1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1,0,1],
[1,1,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,0,1],
[1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
]
    path_spot_raw = multi_Astar(map1, start, end, pos)
    path_spot = []
    spot_ori = []

    for i in range(0, len(path_spot_raw)):
        path_spot.append((path_spot_raw[0]*40-20, path_spot_raw[1]*40-20))
        spot_ori.append((1.0, 0.0)) if path_spot_raw[i+1][0] > path_spot_raw[i][0] else spot_ori.append((0.0, 1.0))
        spot_ori.append((0.707,0.707)) if path_spot_raw[i+1][1] > path_spot_raw[i][1] else spot_ori.append((-0.707,0.707))
    '''TODO finish the transfrom of 10x10 Map path to the real world path'''



    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'
    
    for i in range(0, len(path_spot)):
        pose_sramped = PoseStamped()
        pose_sramped.pose.position.x = path_spot[i][0]
        pose_sramped.pose.position.y = path_spot[i][1]
        pose_sramped.pose.orientation.w = spot_ori[i][0]
        pose_sramped.pose.orientation.z = spot_ori[i][1]

        path_msg.poses.append(pose_sramped)
    plan_publisher.publish(path_msg)
def receive_treasure_location_callback(data):
    treasure_str = data.data
    treasure_pos = json.loads(treasure_str)
    Magic(treasure_pos)
if __name__ == '__main__':
    rospy.init_node('plan_publisher')
    plan_publisher = rospy.Publisher('/planned_path', Path, queue_size=10)
    treasure_pos_sub = rospy.Subscriber('treasure_location', String, receive_treasure_location_callback)
    rospy.spin()

    