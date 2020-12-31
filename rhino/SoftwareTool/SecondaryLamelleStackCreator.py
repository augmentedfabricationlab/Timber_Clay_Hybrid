import random

stack = []
for i in range(900):
    number = random.random()
    number *= 30

    number += 2

    stack.append(number)
print(stack)
