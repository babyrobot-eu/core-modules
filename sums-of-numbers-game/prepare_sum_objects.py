import pickle
from random import shuffle

sums = [(5, 105) , (205, 305), (405, 1005), (1105, 1205), (1305, 1405)]

shuffle(sums)
a = {'sums': sums, 'current_sum': 0}
with open('child_data/child1.pkl', 'wb') as f:
    pickle.dump(obj=a, file=f)
print(a)
shuffle(sums)
b = {'sums': sums, 'current_sum': 0}
with open('child_data/child2.pkl', 'wb') as f:
    pickle.dump(obj=b, file=f)
print(b)
shuffle(sums)
c = {'sums': sums, 'current_sum': 0}
with open('child_data/child3.pkl', 'wb') as f:
    pickle.dump(obj=c, file=f)
print(c)
