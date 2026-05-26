
import os
import cv2
import pandas as pd
import numpy as np
from glob import glob
import argparse
from tqdm import tqdm

import matplotlib.pyplot as plt

import torch
import torchvision
from torchvision.models.detection.rpn import AnchorGenerator


def get_model(num_keypoints, weights_path=None, device=None):
    
    anchor_generator = AnchorGenerator(sizes=(32, 64, 128, 256, 512), aspect_ratios=(0.25, 0.5, 0.75, 1.0, 2.0, 3.0, 4.0))
    model = torchvision.models.detection.keypointrcnn_resnet50_fpn(pretrained=False,
                                                                   pretrained_backbone=True,
                                                                   num_keypoints=num_keypoints,
                                                                   num_classes = 2, # Background is the first class, object is the second class
                                                                   rpn_anchor_generator=anchor_generator)
    model.to(device)
    if weights_path:
        state_dict = torch.load(weights_path, map_location=device)
        model.load_state_dict(state_dict)      
        print(f"Model loaded =>{weights_path}")  
        
    return model


def return_pressure(main_keypoints, min_value=0, max_value=3.5):
    min_reading        = main_keypoints[0]
    max_reading        = main_keypoints[1]
    center             = main_keypoints[2]
    tip                = main_keypoints[3]

    c = np.sqrt(((center[0] - min_reading[0])**2 + (center[1] - min_reading[1])**2))
    b = np.sqrt((center[0] - tip[0])**2 + (center[1] - tip[1])**2)
    a = np.sqrt((min_reading[0] - tip[0])**2 + (min_reading[1] - tip[1])**2)

    d = np.sqrt((max_reading[0] - center[0])**2 + (max_reading[1] - center[1])**2)
    e = np.sqrt((min_reading[0] - max_reading[0])**2 + (min_reading[1] - max_reading[1])**2)

    theta = (np.arccos((b**2 + c**2 - a**2)/(2*b*c)) / np.pi) * 180
    phi   = (np.arccos((c**2 + d**2 - e**2)/(2*c*d)) / np.pi) * 180

    sigma = 360 - phi

    pointed_value = np.round((theta / sigma) * (max_value - min_value), 2)
    
    return pointed_value

def find_hit(gt, pred, threshold=0.2):
    if abs(gt - pred) > threshold:
        return False
    else:
        return True



def visualize(image, keypoints, image_original=None, bboxes_original=None, keypoints_original=None):
    keypoints_classes_ids2names = {0: 'm', 1: 'M', 2:'c', 3:'t'}
    fontsize = 2

    for kps in keypoints:
        for idx, kp in enumerate(kps):
            kp    = tuple([int(i) for i in kp[:2]])
            image = cv2.circle(image.copy(), kp, 5, (255,0,0), 10)
            image = cv2.putText(image.copy(), " " + keypoints_classes_ids2names[idx], tuple(kp), cv2.FONT_HERSHEY_SIMPLEX, fontsize, (255,0,0), 3, cv2.LINE_AA)

    print(os.getcwd())
    cv2.imwrite('static/files/final_output.jpg', image)

def predict_gauge_meter(model_path, image_path):
    
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    device = 'cpu'
    model  = get_model(num_keypoints = 4, weights_path = model_path, device=device)
   

    orig   = cv2.imread(f"{image_path}")
    img    = orig / 255.0
    
    images = torch.from_numpy(img)
    images = torch.permute(images, (2, 0, 1))
    if device != 'cpu': images = images.cuda()
    images = [images.float()]

    with torch.no_grad():
        model.to(device)
        model.eval()
        output = model(images)

    readings, scores = [], []

    for score,  keypoints in zip(output[0]['keypoints_scores'].cpu().numpy(), output[0]['keypoints'].cpu().numpy()):
        readings.append(keypoints)
        scores.append(score)


    scores = np.array(scores).T
    index  = np.argmax(scores, axis=1)

    key_scores = []
    for key_num, ind in enumerate(index):
        key_scores.append(list(readings[ind][key_num]))

    print(key_scores)
    visualize(orig, [key_scores])
    predict_pointer = return_pressure(key_scores)

    return predict_pointer



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Provide required fields')

    parser.add_argument('--image_path', help='image storage location', required=True)
    parser.add_argument('--model_path', help='Trained model path', required=True)
    parser.add_argument('--val_file', help='validation ground truth csv')
  
    args = parser.parse_args()

    hit_count_1  = 0
    hit_count_2  = 0

    print(args.image_path)
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model = get_model(num_keypoints = 4, weights_path = args.model_path)
    model.to(device)

    if args.val_file:
        test_pd    = pd.read_csv(args.val_file)
        results    = pd.DataFrame(columns=['img_name', 'RealValue', 'PredictedValue', 'Error', 'Hit(E<0.15)', 'Hit(E<0.2)'])

        total      = len(test_pd)

        for (_, row) in tqdm(test_pd.iterrows()):
            img_name  = row['x']
            realvalue = row['t']
            
            predict_pointer = predict_gauge_meter(model, args.image_path, img_name)
            hit1            = find_hit(realvalue, predict_pointer, 0.15)
            hit2            = find_hit(realvalue, predict_pointer, 0.2)


            if hit1:
                hit_count_1 += 1
                hit_count_2 += 1

            elif hit2:
                hit_count_2 += 1
            
            row_ = {'img_name':img_name, 'RealValue':realvalue, 'PredictedValue':predict_pointer, 'Error':abs(predict_pointer - realvalue), 'Hit(E<0.15)':hit1, 'Hit(E<0.2)':hit2}
            results = results.append(row_, ignore_index=True)

        print(f"Total=>{total} | hit_count_1=>{hit_count_1} | hit_count_2 {hit_count_2} | Hit(E<0.15)=>{(hit_count_1 / total)*100} | Hit(E<0.15)=>{(hit_count_2 / total)*100}")

    else:
        results    = pd.DataFrame(columns=['img_name', 'PredictedValue'])
        test_file = glob(f"{args.image_path}/*")

        for img_path in tqdm(test_file):
            img_name = img_path.split('/')[-1]
            predict_pointer = predict_gauge_meter(model, args.image_path, img_name)
            row_ = {'img_name':img_name, 'PredictedValue':predict_pointer}
            results = results.append(row_, ignore_index=True)


    results.to_csv('./gauge_meter_results.csv', index=False)









    