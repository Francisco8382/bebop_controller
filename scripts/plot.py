#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
import os
import csv
import time
import pandas as pd
import matplotlib
matplotlib.use('Agg')
#import matplotlib.pyplot as plt
#import sys

## @file plot.py
## @brief Node file for plotting test results.
##
## This node requires the following parameters to run.
## @param Dir The directory where the CSV files are saved.
## @param /Subfolder The subfolder where the CSV file is located.
## @param Topics/CSV_End Topic used to communicate when the trajectory begins and the *data_to_csv* node should stop saving the data.
## @param YAML Topic used to specify which YAML file corresponds to the running controller and to store the gain values by copying this file.
## It is recommended to pass the *Topics/CSV_End* parameter using the following YAML file.
## - topics.yaml

## Graph generator class.
class Plots:
    def __init__(self, path, topic, yaml):
        self.path = path
        self.yaml = yaml
        rospy.Subscriber(topic, Empty, self.Plot)
        rospy.spin() 

    ## Function that generates the graphs of the test results.
    def Plot(self, data):
        data = pd.read_csv(os.path.join(self.path,'data.csv'))
        print(os.path.join(self.path,'data.csv'))
        col={'x_ref': r'$x_r$', 'y_ref': r'$y_r$', 'z_ref': r'$z_r$', 'yaw_ref': r'$\psi_r$',
                'x': r'$x_d$', 'y': r'$y_d$', 'z': r'$z_d$', 'yaw': r'$\psi_d$',
                'v_x': r'$v_x$', 'v_y': r'$v_y$', 'v_z': r'$v_z$', 'v_yaw': r'$v_\psi$'}
        data.rename(columns=col, inplace = True)
        drone_df = data.loc[:, ['Time',r'$x_d$',r'$y_d$',r'$z_d$',r'$\psi_d$']]
        reference_df = data.loc[:, ['Time',r'$x_r$',r'$y_r$',r'$z_r$',r'$\psi_r$']]
        cmd_vel_df = data.loc[:, ['Time',r'$v_x$',r'$v_y$',r'$v_z$',r'$v_\psi$']]

        minV = drone_df['Time'].min()
        maxV = drone_df['Time'].max()

        ax = drone_df.plot(x=0,grid=True,title='Drone')
        ax.set_xlim(minV,maxV)
        ax.get_figure().savefig(os.path.join(self.path,'drone.png'))
        ax = reference_df.plot(x=0,grid=True,title='Reference')
        ax.set_xlim(minV,maxV)
        ax.get_figure().savefig(os.path.join(self.path,'reference.png'))
        ax = cmd_vel_df.plot(x=0,grid=True,title='CMD_Vel')
        ax.set_xlim(minV,maxV)
        ax.get_figure().savefig(os.path.join(self.path,'cmd_vel.png'))
        
        for ax in [r'x',r'y',r'z',r'\psi']:
            ax_orig = r'$' + ax + r'$'
            ax_drone = r'$' + ax + r'_d$'
            ax_ref = r'$' + ax + r'_r$'
            df = data.loc[:,['Time',ax_drone, ax_ref]]
            name = ax + '.png'
            fn = os.path.join(self.path,name)
            ax = df.plot(x=0,grid=True,title=ax_orig)
            ax.set_xlim(minV,maxV)
            ax.get_figure().savefig(fn)

        os.system("cp $(rospack find bebop_controller)/resource/{} {}".format(self.yaml, self.path))
        os.system("cp $(rospack find bebop_controller)/resource/max_speed.yaml {}".format(self.path))
        os.system("cp $(rospack find bebop_controller)/resource/safe_zone.yaml {}".format(self.path))

        rospy.signal_shutdown("")

if __name__ == '__main__':
    rospy.init_node('plot')
    path = rospy.get_param('~Dir')
    sub = rospy.get_param('/Subfolder')
    topic = rospy.get_param('~Topics/CSV_End')
    yaml = rospy.get_param('~YAML')
    sub = sub[:-1]
    plt = Plots(os.path.join(path,sub),topic,yaml)
    
    