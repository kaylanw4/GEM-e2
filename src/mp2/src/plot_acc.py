import matplotlib.pyplot as plt

acc= []
with open('acc.txt', 'r') as f:
    i = 0
    for line in f:
        if i % 1 == 0:
            acc.append(float(line))
        i += 1

time = [i*0.01 for i in range(len(acc))]
# breakpoint()
        
plt.figure()

plt.scatter(time, acc, color="blue", s=2)

# breakpoint()
plt.xlim((0, 150))
plt.ylim((-6, 6))
plt.xlabel("Time (s)")
plt.ylabel("Acceleration")
# print([round(i, 0) for i in range(-200, 350, 50)])
plt.xticks([round(i, 0) for i in range(0, 150, 30)])
plt.yticks([round(i, 0) for i in range(-6, 6, 2)])
plt.savefig("acc.png")


