from parameters import shrink_factor, image_width, image_height
from model import SegNet, load_model_json
from torchvision.io import read_image
import torch.nn.functional as F
import torchvision
import torch



checkpoint = torch.load("/home/borischeng/Robocar/mask_generator/weights/checkpoint_pavements_20250627_143321.pth.tar", weights_only=True)
model_json = load_model_json()
model = SegNet(in_chn=model_json['in_chn'], out_chn=model_json['out_chn'], BN_momentum=model_json['bn_momentum'])
model.load_state_dict(checkpoint["state_dict"])
model.eval()



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


def transform_image(image_name, debug = False):
    images = torch.zeros((1, 3, to_even(image_height/shrink_factor), to_even(image_width/shrink_factor)))
    images[0] = downscale(read_image(image_name), factor=shrink_factor)[:, :3, :, :]
    output = model(images)
    
    res = torch.argmax(output, dim=1).type(torch.long)
    
    if debug:
        show_image(res[0].to(torch.float32))
    
    return res[0]



if __name__ =="__main__":
    transform_image("car_pictures/320_180/frame_07775.png", debug=True)
