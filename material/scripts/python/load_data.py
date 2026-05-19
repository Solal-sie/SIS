#!/usr/bin/env python3

from matplotlib import pyplot as plt
import pandas as pd

def load_file(filename: str) -> pd.DataFrame:
    '''
    Load a csv file into a pandas dataframe and strip spaces from column names

    Parameters
    ----------
    - `filename`: path to the csv file

    Returns
    -------
    - `df`: pandas dataframe
    '''

    # Load the data 
    df = pd.read_csv(filename, sep=',')

    # strip spaces from column names 
    df.rename(columns=lambda x: x.strip(), inplace=True)

    # drop the last column (empty)
    df = df.drop(df.columns[-1], axis=1)

    return df

def plot_pose_groundtruth(df:pd.DataFrame):
    '''
    Plot the pose ground truth
    '''

    fig = plt.figure(figsize=(6,5))
    shape = (2,2)

    ax = plt.subplot2grid(shape, (0,0))
    ax.set_title('Position')
    ax.plot(df.x, df.y)
    l_min = min(*ax.get_xbound(), *ax.get_ybound())
    l_max = max(*ax.get_xbound(), *ax.get_ybound())
    ax.set_xlim(l_min,l_max)
    ax.set_ylim(l_min,l_max)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid()

    ax = plt.subplot2grid(shape, (0,1))
    ax.set_title('Heading')
    ax.plot(df.time, df.heading)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('[rad]')
    ax.grid()
    
    ax = plt.subplot2grid(shape, (1,0))
    ax.set_title('Velocity')
    ax.plot(df.time, df.vel_x, label='$\dot{x}$')
    ax.plot(df.time, df.vel_y, label='$\dot{y}$')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('[m/s]')
    ax.legend()
    ax.grid()

    ax = plt.subplot2grid(shape, (1,1))
    ax.set_title('Angular velocity')
    ax.plot(df.time, df.vel_heading)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('[rad/s]')
    ax.grid()

    plt.tight_layout()


if __name__ == '__main__':

    df = load_file('controllers/supervisor/data/ground_truth.csv')
    print(df.head())
    
    plot_pose_groundtruth(df)

    plt.show()