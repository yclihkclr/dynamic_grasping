import pickle
import random

import numpy as np
import torch
from torch import nn
import matplotlib.pyplot as plt
from torch.utils.data import Dataset,DataLoader
from tqdm import *
import argparse
from torch.optim.lr_scheduler import StepLR
import copy
import time
import glob

import math

# Define LSTM Neural Networks
class LstmRNN(nn.Module):
    """
        Parameters：
        - input_size: feature size
        - hidden_size: number of hidden units
        - output_size: number of output
        - num_layers: layers of LSTM to stack
    """
    def __init__(self, input_size=1, hidden_size=1, output_size=1, num_layers=1):
        super().__init__()

        # self.embedding=nn.Linear(input_size,hidden_size)
 
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers,batch_first=True) # utilize the LSTM model in torch.nn

        self.forwardCalculation = nn.Linear(hidden_size, output_size)

        # self.linear=nn.Linear(input_size,hidden_size)
 
    def forward(self, _x):
        # feature=self.embedding(_x)
        x, _ = self.lstm(_x)  # _x is input, size (batch, seg_len, input_size)
        b, s, h = x.shape  # x is output, size (batch, seg_len, input_size)
        # x = x.view(s*b, h)
        x = self.forwardCalculation(x)
        # x = x.view(s, b, -1)
        x=x[:,-1,:]
        return x

def distance_function(dataset):
    m=len(dataset)
    distance=[0]
    for i in range(m-1):

        delta_dis=math.sqrt((dataset[i][0] - dataset[i+1][0]) ** 2 + (dataset[i][1] - dataset[i+1][1]) ** 2)
        distance.append(delta_dis)

    distance_num=np.expand_dims(np.array(distance),1)

    return distance_num

def angle_function(dataset):
    m=len(dataset)
    angle=[0]
    for i in range(m-1):
        delta_ang=math.atan2((dataset[i+1][1] - dataset[i][1]),(dataset[i+1][0] - dataset[i][0]))
        angle.append(delta_ang)

    angle_num=np.expand_dims(np.array(angle),1)

    return angle_num

def model_select(pose_xy,interval):
  
    dataset_ = []
    m = int(int(len(pose_xy)) / interval)
    for i in range(m):
        dataset_.append(pose_xy[i * interval])

    angle = angle_function(dataset_)
    model_path=detect_motion(angle)

    return model_path

def get_val_loss(model, Val):
    val_loss=[]
    model.eval()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    loss_function = nn.MSELoss().to(device)

    for (seq, label) in Val:
        seq = seq.to(device)
        label = label.to(device)
        y_pred = model(seq)
        loss = loss_function(y_pred, label)
        val_loss.append(loss.item())
    size=len(val_loss)
    avg_val_loss=np.sum(val_loss)/size

    return avg_val_loss

def motion_pred(pose_xy):
    model_path=model_select(pose_xy, interval=1)
    print('loading prediction models...')

    INPUT_FEATURES_NUM = 2
    OUTPUT_FEATURES_NUM = 2
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = LstmRNN(INPUT_FEATURES_NUM, 32, output_size=OUTPUT_FEATURES_NUM, num_layers=2)  # 16 hidden units
    model.load_state_dict(torch.load(model_path)['models'])
    model.to(device)
    model.eval()
    print('predicting...')

    start_time=time.time()
    pose_xy=torch.FloatTensor(pose_xy)
    pose_xy = pose_xy.to(device)
    pose_pred_seq=torch.zeros(10,2)

    with torch.no_grad():
        for i in range(10):
            pose_xy = torch.unsqueeze(pose_xy, 0)
            pose_pred = model(pose_xy)
            # print("length of pose:",len(pose_xy))

            pose_xy = torch.squeeze(pose_xy, 0)
            pose_xy=torch.cat((pose_xy[1:,:],pose_pred),axis=0)
           # y_pred_seq[i]=torch.cat((y_pred_seq,y_pred),axis=0)
            pose_pred_seq[i]=pose_pred

    end_time=time.time()
    t_last=end_time-start_time

    # with torch.no_grad():
        
    #     pose_xy = torch.unsqueeze(pose_xy, 0)
    #     y_pred = model(pose_xy)

        # seq = torch.squeeze(seq, 0)

    return pose_pred_seq,t_last


def detect_motion(angle):
    # print("angle is:",angle)
    # if (abs(angle[1]-angle[9])>(math.pi/6)):
    #     model_path='src/prediction_client/prediction_client/motion_prediction/best_model_circular.pt'
    #     print('detected circular motion')
    # else:
    model_path='src/prediction_client/prediction_client/motion_prediction/best_model_linear.pt'
    print('detected linear motion')
    return model_path