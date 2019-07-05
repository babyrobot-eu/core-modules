import pickle

with open('child_data/child1.pkl', 'rb') as f:
    d = pickle.load(f)
print(d)