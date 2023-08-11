import numpy as np
import matplotlib.pyplot as plt

recv_time_profile = np.loadtxt('recv_time.txt', delimiter=',')
recv_time_profile1 = np.loadtxt('recv_time_1.txt')
recv_time_profile2 = np.loadtxt('recv_time_2.txt')
recv_time_profile3 = np.loadtxt('recv_time_3.txt')
recv_time_profile4 = np.loadtxt('recv_time_4.txt')
recv_time_profile5 = np.loadtxt('recv_time_5.txt')

plt.subplot(611)
plt.plot(recv_time_profile[:, 0])
plt.plot(recv_time_profile[:, 1])
print(np.mean(recv_time_profile[:, 0]), np.max(recv_time_profile[:, 0]), np.min(recv_time_profile[:, 0]))
print(np.mean(recv_time_profile[:, 1]), np.max(recv_time_profile[:, 1]), np.min(recv_time_profile[:, 1]))
print()

plt.subplot(612)
plt.plot(recv_time_profile1)
print(np.mean(recv_time_profile1), np.max(recv_time_profile1), np.min(recv_time_profile1))

plt.subplot(613)
plt.plot(recv_time_profile2)
print(np.mean(recv_time_profile2), np.max(recv_time_profile2), np.min(recv_time_profile2))

plt.subplot(614)
plt.plot(recv_time_profile3)
print(np.mean(recv_time_profile3), np.max(recv_time_profile3), np.min(recv_time_profile3))

plt.subplot(615)
plt.plot(recv_time_profile4)
print(np.mean(recv_time_profile4), np.max(recv_time_profile4), np.min(recv_time_profile4))

plt.subplot(616)
plt.plot(recv_time_profile5)
print(np.mean(recv_time_profile5), np.max(recv_time_profile5), np.min(recv_time_profile5))



exec_delay1 = np.loadtxt('exec_delay_1.txt', delimiter=',')
exec_delay2 = np.loadtxt('exec_delay_2.txt', delimiter=',')
exec_delay3 = np.loadtxt('exec_delay_3.txt', delimiter=',')
exec_delay4 = np.loadtxt('exec_delay_4.txt', delimiter=',')
exec_delay5 = np.loadtxt('exec_delay_5.txt', delimiter=',')

plt.figure()
plt.subplot(411)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 1], 'o', markersize=1, label='IPC Command')
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 7], 'o', markersize=1, label='Robot Feedback')
delay_list1 = []
for i in range(exec_delay1.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay1.shape[0]):
        diff = abs(exec_delay1[i, 1] - exec_delay1[j, 7])
        if((diff < min_diff and diff > 0.05) and j != exec_delay1.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list1.append(delay)
            break
plt.plot(exec_delay1[:len(delay_list1), 0]-exec_delay1[0, 0], delay_list1, label='Delay')
plt.legend()


plt.subplot(412)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 2], 'o', markersize=1)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 8], 'o', markersize=1)
delay_list1 = []
for i in range(exec_delay1.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay1.shape[0]):
        diff = abs(exec_delay1[i, 2] - exec_delay1[j, 8])
        if((diff < min_diff and diff > 0.05) and j != exec_delay1.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list1.append(delay)
            break
plt.plot(exec_delay1[:len(delay_list1), 0]-exec_delay1[0, 0], delay_list1)

plt.subplot(413)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 3], 'o', markersize=1)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 9], 'o', markersize=1)
delay_list1 = []
for i in range(exec_delay1.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay1.shape[0]):
        diff = abs(exec_delay1[i, 3] - exec_delay1[j, 9])
        if((diff < min_diff and diff > 0.05) and j != exec_delay1.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list1.append(delay)
            break
plt.plot(exec_delay1[:len(delay_list1), 0]-exec_delay1[0, 0], delay_list1)

plt.subplot(414)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 5], 'o', markersize=1)
plt.plot(exec_delay1[:, 0]-exec_delay1[0, 0], exec_delay1[:, 11], 'o', markersize=1)
delay_list1 = []
for i in range(exec_delay1.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay1.shape[0]):
        diff = abs(exec_delay1[i, 5] - exec_delay1[j, 11])
        if((diff < min_diff and diff > 0.05) and j != exec_delay1.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list1.append(delay)
            break
plt.plot(exec_delay1[:len(delay_list1), 0]-exec_delay1[0, 0], delay_list1)


plt.figure()
plt.subplot(411)
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 1])
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 7])
delay_list = []
for i in range(exec_delay2.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay2.shape[0]):
        diff = abs(exec_delay2[i, 1] - exec_delay2[j, 7])
        if((diff < min_diff and diff > 0.05) and j != exec_delay2.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay2[:len(delay_list), 0]-exec_delay2[0, 0], delay_list)

plt.subplot(412)
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 2])
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 8])
delay_list = []
for i in range(exec_delay2.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay2.shape[0]):
        diff = abs(exec_delay2[i, 2] - exec_delay2[j, 8])
        if((diff < min_diff and diff > 0.05) and j != exec_delay2.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay2[:len(delay_list), 0]-exec_delay2[0, 0], delay_list)

plt.subplot(413)
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 3])
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 9])
delay_list = []
for i in range(exec_delay2.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay2.shape[0]):
        diff = abs(exec_delay2[i, 3] - exec_delay2[j, 9])
        if((diff < min_diff and diff > 0.05) and j != exec_delay2.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay2[:len(delay_list), 0]-exec_delay2[0, 0], delay_list)

plt.subplot(414)
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 5])
plt.plot(exec_delay2[:, 0]-exec_delay2[0, 0], exec_delay2[:, 11])
delay_list = []
for i in range(exec_delay2.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay2.shape[0]):
        diff = abs(exec_delay2[i, 5] - exec_delay2[j, 11])
        if((diff < min_diff and diff > 0.05) and j != exec_delay2.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay2[:len(delay_list), 0]-exec_delay2[0, 0], delay_list)

plt.figure()
plt.subplot(411)
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 1])
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 7])
delay_list = []
for i in range(exec_delay3.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay3.shape[0]):
        diff = abs(exec_delay3[i, 1] - exec_delay3[j, 7])
        if((diff < min_diff and diff > 0.05) and j != exec_delay3.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay3[:len(delay_list), 0]-exec_delay3[0, 0], delay_list)

plt.subplot(412)
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 2])
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 8])
delay_list = []
for i in range(exec_delay3.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay3.shape[0]):
        diff = abs(exec_delay3[i, 2] - exec_delay3[j, 8])
        if((diff < min_diff and diff > 0.05) and j != exec_delay3.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay3[:len(delay_list), 0]-exec_delay3[0, 0], delay_list)

plt.subplot(413)
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 3])
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 9])
delay_list = []
for i in range(exec_delay3.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay3.shape[0]):
        diff = abs(exec_delay3[i, 3] - exec_delay3[j, 9])
        if((diff < min_diff and diff > 0.05) and j != exec_delay3.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay3[:len(delay_list), 0]-exec_delay3[0, 0], delay_list)

plt.subplot(414)
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 5])
plt.plot(exec_delay3[:, 0]-exec_delay3[0, 0], exec_delay3[:, 11])
delay_list = []
for i in range(exec_delay3.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay3.shape[0]):
        diff = abs(exec_delay3[i, 5] - exec_delay3[j, 11])
        if((diff < min_diff and diff > 0.05) and j != exec_delay3.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay3[:len(delay_list), 0]-exec_delay3[0, 0], delay_list)

plt.figure()
plt.subplot(411)
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 1])
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 7])
delay_list = []
for i in range(exec_delay4.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay4.shape[0]):
        diff = abs(exec_delay4[i, 1] - exec_delay4[j, 7])
        if((diff < min_diff and diff > 0.05) and j != exec_delay4.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay4[:len(delay_list), 0]-exec_delay4[0, 0], delay_list)

plt.subplot(412)
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 2])
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 8])
delay_list = []
for i in range(exec_delay4.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay4.shape[0]):
        diff = abs(exec_delay4[i, 2] - exec_delay4[j, 8])
        if((diff < min_diff and diff > 0.05) and j != exec_delay4.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay4[:len(delay_list), 0]-exec_delay4[0, 0], delay_list)


plt.subplot(413)
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 3])
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 9])
delay_list = []
for i in range(exec_delay4.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay4.shape[0]):
        diff = abs(exec_delay4[i, 3] - exec_delay4[j, 9])
        if((diff < min_diff and diff > 0.05) and j != exec_delay4.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay4[:len(delay_list), 0]-exec_delay4[0, 0], delay_list)

plt.subplot(414)
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 5])
plt.plot(exec_delay4[:, 0]-exec_delay4[0, 0], exec_delay4[:, 11])
delay_list = []
for i in range(exec_delay4.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay4.shape[0]):
        diff = abs(exec_delay4[i, 5] - exec_delay4[j, 11])
        if((diff < min_diff and diff > 0.05) and j != exec_delay4.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay4[:len(delay_list), 0]-exec_delay4[0, 0], delay_list)

plt.figure()
plt.subplot(411)
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 1])
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 7])
delay_list = []
for i in range(exec_delay5.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay5.shape[0]):
        diff = abs(exec_delay5[i, 1] - exec_delay5[j, 7])
        if((diff < min_diff and diff > 0.05) and j != exec_delay5.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay5[:len(delay_list), 0]-exec_delay5[0, 0], delay_list)

plt.subplot(412)
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 2])
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 8])
delay_list = []
for i in range(exec_delay5.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay5.shape[0]):
        diff = abs(exec_delay5[i, 2] - exec_delay5[j, 8])
        if((diff < min_diff and diff > 0.05) and j != exec_delay5.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay5[:len(delay_list), 0]-exec_delay5[0, 0], delay_list)

plt.subplot(413)
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 3])
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 9])
delay_list = []
for i in range(exec_delay5.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay5.shape[0]):
        diff = abs(exec_delay5[i, 3] - exec_delay5[j, 9])
        if((diff < min_diff and diff > 0.05) and j != exec_delay5.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay5[:len(delay_list), 0]-exec_delay5[0, 0], delay_list)

plt.subplot(414)
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 5])
plt.plot(exec_delay5[:, 0]-exec_delay5[0, 0], exec_delay5[:, 11])
delay_list = []
for i in range(exec_delay5.shape[0]):
    min_diff = 10000
    delay = 8
    for j in range(i+1, exec_delay5.shape[0]):
        diff = abs(exec_delay5[i, 5] - exec_delay5[j, 11])
        if((diff < min_diff and diff > 0.05) and j != exec_delay5.shape[0]-1):
            delay = abs(i-j)*8
            min_diff = diff
        else:
            delay_list.append(delay)
            break
plt.plot(exec_delay5[:len(delay_list), 0]-exec_delay5[0, 0], delay_list)
plt.legend()
plt.show()