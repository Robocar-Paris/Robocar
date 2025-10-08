from parameters import shrink_factor, image_width, image_height
from model import SegNet, load_model_json
from torchvision.io import read_image
from datetime import datetime
import torchvision
import torch.nn.functional as F
import torch.optim as optim
import torch.nn as nn
import torch
import json
import os



trans = torchvision.transforms.ToPILImage()
def show_image(img):
    out = trans(img.cpu())
    out.show()


def to_even(val):
    return int(val + 1) if int(val) % 2 == 1  else int(val)


def downscale(images, factor=1):
    if len(images.shape) == 3:
        images = images.reshape((1,) + images.shape)
    return F.interpolate(images, (to_even(image_height/factor), to_even(image_width/factor)))


def save_checkpoint(state, path):
    torch.save(state, path)
    print("Checkpoint saved at {}".format(path))



train_picture_path = "../DatasetSimuator/ColoredCamera/"
train_mask_path = "../DatasetSimuator/MaskCamera"

model_json = load_model_json()
model = SegNet(in_chn=model_json['in_chn'], out_chn=model_json['out_chn'], BN_momentum=model_json['bn_momentum'])
checkpoint = torch.load("/home/borischeng/Robocar/mask_generator/weights/checkpoint_pavements_20250627_141316.pth.tar", weights_only=True)
model.load_state_dict(checkpoint["state_dict"])
model.eval()

optimizer = optim.SGD(model.parameters(), lr=model_json['learning_rate'], momentum=model_json['sgd_momentum'])
loss_fn = nn.CrossEntropyLoss(weight=torch.tensor(model_json['cross_entropy_loss_weights']))


# checkpoint = torch.load("/home/laiheau/project/robocar/Robocar/mask_generator/weights/checkpoint_pavements_20250627_140907.pth.tar", weights_only=True)
# model.load_state_dict(checkpoint["state_dict"])
# model.eval()


cuda_available = True # torch.cuda.is_available()
if cuda_available:
    model.cuda()
    loss_fn.cuda()

# weight_fn = os.path.join(os.getcwd(), "weights/checkpoint_pavements_{}.pth.tar".format(datetime.now().strftime("%Y%m%d_%H%M%S")))
weight_fn = "/home/laiheau/project/robocar/Robocar/mask_generator/weights/checkpoint_pavements_20250627_140907.pth.tar"

epoch = model_json['epochs']
train_picture_name = [os.path.join(train_picture_path, name) for name in os.listdir(train_picture_path)[:10]]
train_mask_name = [os.path.join(train_mask_path, name) for name in os.listdir(train_mask_path)[:10]]



def get_data(count, train_picture_path, train_mask_path, decal=0):
    if len(os.listdir(train_picture_path)) < count + decal:
        print(f"less than {count} image in {train_picture_path}")
        exit(1)
    if len(os.listdir(train_mask_path)) < count + decal:
        print(f"less than {count} image in {train_mask_path}")
        exit(1)
    
    train_picture_name = [os.path.join(train_picture_path, name) for name in os.listdir(train_picture_path)[decal:count+decal]]
    train_mask_name = [os.path.join(train_mask_path, name) for name in os.listdir(train_mask_path)[decal:count+decal]]

    images = torch.zeros((count, 3, to_even(image_height/shrink_factor), to_even(image_width/shrink_factor)))
    for i in range(count):
        images[i] = downscale(read_image(train_picture_name[i]), factor=shrink_factor)[:, :3, :, :]
    
    masks = torch.zeros((count, 3, to_even(image_height/shrink_factor), to_even(image_width/shrink_factor)))
    masks = torch.zeros((count, 2) + images.shape[2:])
    for i in range(count):
        raw_mask = downscale(read_image(train_mask_name[i]), factor=shrink_factor)
        
        threshold = 120
        masks[i, 0, raw_mask[0][0] < threshold] = 1
        masks[i, 1, raw_mask[0][0] >= threshold] = 1
    
    return images, masks



print(torch.cuda.get_device_name(0))
for i in range(epoch):
    print('Epoch {}:'.format(i))
    sum_loss = 0.0
    
    images, masks = get_data(model_json["image_to_load"], train_picture_path, train_mask_path, decal=(model_json["image_to_load"]*i) % (900 - model_json["image_to_load"]))

    if cuda_available:
        images = images.cuda()
        masks = masks.cuda()
    
    for j in range(1):
        # images = read_image(image_path)
        # images = images.reshape((1,) + images.shape).to(torch.float32)
        # images = downscale(images)
        
        # masks = read_image(mask_path)
        # masks = masks.reshape((1,) + masks.shape).to(torch.float32)
        # masks = downscale(masks)
        
        # new_mask = torch.zeros((1, 2) + images.shape[2:])

        # threshold = 240
        # new_mask[0, 0, masks[0][0] < threshold] = 1
        # new_mask[0, 1, masks[0][0] >= threshold] = 1
        # new_mask.reshape((1,) + new_mask.shape)
        
        # print('Memory Usage:')
        # print('Allocated:', round(torch.cuda.memory_allocated(0)/1024**3,1), 'GB')
        # print('Cached:   ', round(torch.cuda.memory_reserved(0)/1024**3,1), 'GB')
        
        optimizer.zero_grad()
        output = model(images)
        
        loss = loss_fn(output, masks)
        res = torch.argmax(output, dim=1).type(torch.long)
        loss.backward()
        optimizer.step()

        # writer.add_scalar('Loss',loss.item()/trainloader.batch_size, j)
        sum_loss += loss.item()

        print('Loss at {} mini-batch: {} epoch {}'.format(j, loss.item() / len(train_picture_name), i))

    print('Average loss @ epoch: {}'.format((sum_loss / (j+1))))

print("Training complete. Saving checkpoint...")
save_checkpoint({'epoch': epoch, 'state_dict': model.state_dict(), 'optimizer' : optimizer.state_dict()}, weight_fn)    

show_image(res[0].to(torch.float32))
