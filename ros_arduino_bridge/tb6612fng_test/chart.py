# encoding=utf-8
import matplotlib.pyplot as plt
from pylab import *  # 支持中文
mpl.rcParams['font.sans-serif'] = ['SimHei']
leftlist = []  
leftout = []
rightlist = []
rightout = []


with open('/home/llb/chart/result') as f:
    
  for line in f:
      if len(line) > 5:
#         print(line)
        input = line.split(" ")[1]
        temp_in = input.split(":")[1]
        
        output = line.split(" ")[4]
        temp_out = output.split(":")[1]
        temp_out = temp_out.replace("\n","")
        print (temp_out)
        if len(temp_in) > 0 : 
            if input.split(":")[0] == 'input_R':
                rightlist.append(int(temp_in))
            else :
                leftlist.append(int(temp_in))
        if len(temp_out) > 0 : 
            if input.split(":")[0] == 'input_R':
                rightout.append(float(temp_out))
            else :
                leftout.append(float(temp_out))
print len(rightlist),len(leftlist)
pwm = range(len(leftlist))
pwm_scaler = range(len(pwm))
# left_pwm = range(0,480,20)
# right_pwm = range(240,0,-10)

time_scaler = range(len(leftlist))


# plt.plot(x, y, 'ro-')
# plt.plot(x, y1, 'bo-')
# pl.xlim(-1, 11)  # 限定横轴的范围
# pl.ylim(-1, 110)  # 限定纵轴的范围
plt.plot(time_scaler, leftlist,  marker='o', mec='r', mfc='w', label='Left_In')
# plt.plot(time_scaler, rightlist, marker='*',  mec='r', mfc='w',  label='Right')
plt.plot(time_scaler, leftout,
          color='red',   # 线颜色
         linewidth=1.0,  # 线宽 
         linestyle='--',  # 线样式
         label='Left_out')
plt.plot(time_scaler, rightlist,  marker='*', mec='r', mfc='w', label='Right_In')
plt.plot(time_scaler, rightout,
          color='black',
         label='Right_out')
plt.legend()  # 让图例生效
plt.xticks(pwm_scaler, pwm, rotation=45)
plt.margins(0)
plt.subplots_adjust(bottom=0.15)
plt.xlabel(u"TIME")  # X轴标签
plt.ylabel("PWM")  # Y轴标签
plt.title("PID")  # 标题

plt.show()
