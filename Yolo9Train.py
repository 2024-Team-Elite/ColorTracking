import torch
from new import model as m 

model = m('custom_yolov9.yaml')

# Comment this out if we use our custom model but for now we vibin
model.load_state_dict(torch.load('yolov9_pretrained.pt'))

train_loader = torch.utils.data.DataLoader(
    train_dataset,
    batch_size=2,  # Smaller batch size
    shuffle=True,
    num_workers=8,
    pin_memory=True
)

optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

for epoch in range(100):  # Number of epochs
    model.train()
    for inputs, targets in train_loader:
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = compute_loss(outputs, targets)
        loss.backward()
        optimizer.step()
    
    print(f"Epoch {epoch + 1}, Loss: {loss.item()}")

# SAVE IT OMG
torch.save(model.state_dict(), 'yolov9_custom_trained.pt')