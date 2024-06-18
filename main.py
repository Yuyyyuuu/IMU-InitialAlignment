import math
import numpy as np
import matplotlib.pyplot as plt
#常量
acc_scale=1.5258789063e-06
gyo_scale=1.0850694444e-07
f=100
g=9.7936174
we=math.degrees(7.292115e-5) #deg单位的地球自转角速度
fai=math.radians(30.531651244) #弧度单位下的纬度
pi=4*math.atan(1)
#存储静态数据的类
class data:
    def __init__(self):
        self.fx = 0
        self.fy = 0
        self.fz = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0
#求列表元素均值
def mean(list):
    m = sum(list) / len(list)
    return m
#求列表list0中每一百个数据的均值并以列表的形式存储
def mean_1s(list0,list):
    # 划分数据并计算均值
    for i in range(0, len(list0), 100):
        subset = list0[i:min(i + 100, len(list0))]
        if len(subset) == 100:  # 确保子集中至少有一个元素
            average = mean(subset)
            list.append(average)
#初始对准核心函数
def initial_alignment(fx,fy,fz,wx,wy,wz):
    # n系下的 g 和 we
    gn = np.array([0, 0, g])
    wen = np.array([we*math.cos(fai), 0, -we*math.sin(fai)])
    # b系下的 g 和 we
    gb = np.array([-fx, -fy, -fz])
    web = np.array([wx, wy, wz])
    #单位正交化得计算所需向量
    vg=gn/np.linalg.norm(gn)
    vw=np.cross(gn,wen)/np.linalg.norm(np.cross(gn,wen))
    vgw=np.cross(np.cross(gn,wen),gn)/np.linalg.norm(np.cross(np.cross(gn,wen),gn))
    wg=gb/np.linalg.norm(gb)
    ww=np.cross(gb,web)/np.linalg.norm(np.cross(gb,web))
    wgw=np.cross(np.cross(gb,web),gb)/np.linalg.norm(np.cross(np.cross(gb,web),gb))
    #排列成矩阵
    A = np.column_stack((vg, vw, vgw))
    #print(A)
    B = np.row_stack((wg, ww, wgw))
    #print(B)
    #计算姿态矩阵
    C=np.dot(A, B)
    #print(C)
    #转化为姿态角
    yaw=math.atan2(C[1,0],C[0,0])
    roll=math.atan2(C[2,1],C[2,2])
    pitch=math.atan2(-C[2,0],math.sqrt(C[2,1]*C[2,1]+C[2,2]*C[2,2]))
    if (pitch > ( pi / 2 )):
        pitch=pitch-pi
    elif (pitch < -(pi / 2)):
        pitch = pitch + pi
    #返回的姿态角单位为deg
    return math.degrees(yaw),math.degrees(pitch),math.degrees(roll)

datas=[]

#读入文件
filepath = r'D:\惯性导航编程作业\实验二 初始对准\data\静初始对准.ASC'
with open(filepath, 'r') as file:
    for line in file:
        if line.startswith("%RAWIMUSA"):
            obj=data()
            line_data = line.strip().split(',')
            #读入为比力，单位m/s2
            obj.fy = float(line_data[7]) * acc_scale * f
            obj.fx = -float(line_data[6]) * acc_scale * f
            obj.fz = -float(line_data[5]) * acc_scale * f
            #读入角速度，单位deg/s
            stri = line_data[10].split("*")
            obj.wy=math.degrees(float(stri[0]) * gyo_scale * f)
            obj.wx=math.degrees(-float(line_data[9]) * gyo_scale * f)
            obj.wz=-math.degrees(float(line_data[8]) * gyo_scale * f)
            datas.append(obj)

#task1:整段静态数据平均后计算一次姿态角
#取出列表
total_fx = [value.fx for value in datas]
total_fy = [value.fy for value in datas]
total_fz = [value.fz for value in datas]
total_wx = [value.wx for value in datas]
total_wy = [value.wy for value in datas]
total_wz = [value.wz for value in datas]
#计算整体均值
mean_total_fx=mean(total_fx)
mean_total_fy=mean(total_fy)
mean_total_fz=mean(total_fz)
mean_total_wx=mean(total_wx)
mean_total_wy=mean(total_wy)
mean_total_wz=mean(total_wz)

total_yaw,total_pitch,total_roll=initial_alignment(mean_total_fx,mean_total_fy,mean_total_fz,mean_total_wx,mean_total_wy,mean_total_wz)
print("整段静态数据平均后计算所得姿态角：（航向角/deg、俯仰角/deg、横滚角/deg）")
print(total_yaw,total_pitch,total_roll)


#task2:每秒平均值计算姿态角，并画出“姿态角-时间”曲线
#获得每1s数据的均值列表
aver1s_fx=[]
aver1s_fy=[]
aver1s_fz=[]
aver1s_wx=[]
aver1s_wy=[]
aver1s_wz=[]
mean_1s(total_fx,aver1s_fx)
mean_1s(total_fy,aver1s_fy)
mean_1s(total_fz,aver1s_fz)
mean_1s(total_wx,aver1s_wx)
mean_1s(total_wy,aver1s_wy)
mean_1s(total_wz,aver1s_wz)
#进行姿态解算并将姿态角存储在对应的三个列表中
aver1s_y=[]
aver1s_p=[]
aver1s_r=[]
aver1s_t=[]
for i in range(len(aver1s_fx)):
    y,p,r=initial_alignment(aver1s_fx[i],aver1s_fy[i],aver1s_fz[i],aver1s_wx[i],aver1s_wy[i],aver1s_wz[i])
    aver1s_y.append(y)
    aver1s_p.append(p)
    aver1s_r.append(r)
    aver1s_t.append(i)
# 绘制曲线
figure1=plt.figure(figsize=(10,3),num="attitude angels with 1s mean")  # 设置画布的大小
plt.plot(aver1s_t, aver1s_y, label="yaw")  # 绘制 y-t 曲线
plt.plot(aver1s_t, aver1s_p, label="pitch")  # 绘制 p-t 曲线
plt.plot(aver1s_t, aver1s_r, label="roll")  # 绘制 r-t 曲线
# 添加标题和标签
plt.title("attitude angels with 1s mean")  # 设置整个图的标题
plt.xlabel("t/s")  # x 轴标签
plt.ylabel("yaw/deg, p/deg, r/deg")  # y 轴标签
# 添加图例
plt.legend()
# 显示图形
plt.show()


#task3:每历元计算姿态角，并画出“姿态角-时间”曲线
#进行姿态解算并将姿态角存储在对应的三个列表中
total_y=[]
total_p=[]
total_r=[]
total_t=[]

for i in range(len(total_fx)):
    y,p,r=initial_alignment(total_fx[i],total_fy[i],total_fz[i],total_wx[i],total_wy[i],total_wz[i])
    total_y.append(y)
    total_p.append(p)
    total_r.append(r)
    total_t.append(i/f)
# 绘制曲线
figure2=plt.figure(figsize=(10,4),num="attitude angels")  # 设置画布的大小
plt.plot(total_t, total_y, label="yaw")  # 绘制 y-t 曲线
plt.plot(total_t, total_p, label="pitch")  # 绘制 p-t 曲线
plt.plot(total_t, total_r, label="roll")  # 绘制 r-t 曲线
# 添加标题和标签
plt.title("attitude angels")  # 设置整个图的标题
plt.xlabel("t/s")  # x 轴标签
plt.ylabel("yaw/deg, p/deg, r/deg")  # y 轴标签
# 添加图例
plt.legend()
# 显示图形
plt.show()


