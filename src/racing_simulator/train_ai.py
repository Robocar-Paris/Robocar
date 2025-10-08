from lib.RobocarModel import RobocarModel
import torch.optim as optim
import torch.nn as nn
import torch
import pandas as pd
import random



df = pd.read_csv("data_track_2_1_loop_50_rays.csv", header=0, index_col=0)

dataset = torch.tensor(df.values)
X = dataset[:,2:]
y = dataset[:,:2]

X = torch.tensor(X, dtype=torch.float32)
y = torch.tensor(y, dtype=torch.float32).reshape(-1, 2)

model = RobocarModel()

loss_fn   = nn.L1Loss()
optimizer = optim.Adam(model.parameters(), lr=1e-5)

n_epochs = 2000
batch_size = 10

for epoch in range(n_epochs):
    epoch_loss = 0
    for i in range(0, len(X), batch_size):
        Xbatch = X[i:i+batch_size]
        y_pred = model.forward(Xbatch)
        ybatch = y[i:i+batch_size]
        loss = loss_fn(y_pred, ybatch)
        optimizer.zero_grad()
        loss.backward()
        epoch_loss += y_pred.shape[0] * loss.item()
        optimizer.step()
    print(epoch_loss/len(X)/batch_size)


predictions = model.forward(X)

for i in [random.randint(0, len(predictions)) for _ in range(10)]:
    print(f'pred {i} {predictions[i]} rounded {predictions[i].round()} (expected {y[i]})')

torch.save(model.state_dict(), "models/model_trained")
print("model save in models/model_trained")
