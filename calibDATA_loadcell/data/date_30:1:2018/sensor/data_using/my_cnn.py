from scipy import signal
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import LabelEncoder
from scipy import signal


def make_patch_spines_invisible(ax):
    '''
    function used in PLOTTING
    '''
    ax.set_frame_on(True)
    ax.patch.set_visible(False)
    for sp in ax.spines.values():
        sp.set_visible(False)

def plot():
    '''
    PLOTTING stuff
    '''
    fig, ax1 = plt.subplots()
    plt.Figure(figsize=(10,5))
    ax1.plot(data[223:2978,2], 'b-',linewidth=0.5)
    ax1.set_ylabel('Force (16bit)', color='b')
    ax1.tick_params('y', colors='b')

    ax2 = ax1.twinx()
    ax2.plot(resample_baro, 'r-',linewidth=0.5)
    ax2.set_ylabel('Force(N)', color='r')
    ax2.tick_params('y', colors='r')

    ax3 = ax1.twinx()
    # Offset the right spine of par2.  The ticks and label have already been
    # placed on the right by twinx above.
    ax3.spines["right"].set_position(("axes", 1.2))
    # Having been created by twinx, par2 has its frame off, so the line of its
    # detached spine is invisible.  First, activate the frame but make the patch
    # and spines invisible.
    make_patch_spines_invisible(ax3)
    # Second, show the right spine.
    ax3.spines["right"].set_visible(True)
    ax3.plot(resample_ir, 'g-',linewidth=0.5)
    ax3.set_ylabel('IR (16bit)', color='g')
    ax3.tick_params(axis='y', colors='g')

    plt.title('{}degree {}N maxF'.format(angle, maxForce))
    fig.tight_layout()
    plt.show()


angles = [0] # all probing angles
maxForces = [5] # all maxForces
d = {}
# df_empty = pd.DataFrame({'IR' : [], 'baro' : [], 'force' : [], 'degree' : []})
df_empty = pd.DataFrame()
for angle in angles:
    for maxForce in maxForces:
        df = pd.read_excel('{}deg_{}N.xls'.format(angle, maxForce))
        data=df.as_matrix() #converting into numpy array
        ## DATAFIELDSrow_1=IR, row_2=baro, row_3=force, row_4=deg

        ## finding the first occurence of nan to cal length of IR/Force (16 bit)
        len_sensorData = np.where(np.isnan(data[:,2]))[0][1] - 1
        ## resampling MTSdata to match len of IR/baro
        resample_baro = signal.resample(data[439:18130,3],2755)
        # print (len_sensorData)
        # print (f.size)
        resample_ir = signal.resample(data[130:3038,1],2755)


        # df_tmp = {'IR':df['IR (16 bit)'].values[:len_sensorData],'baro':df['Force (16 bit)'].values[:len_sensorData],'force':f, 'degree':df['Degree'].values[:len_sensorData]}
        # data_1 = pd.DataFrame(df_tmp)
        # df_empty = pd.concat([df_empty,data_1])

        # plot()

        # print(df['Force (N)'].values[1:3586])
        shifts = []
        # plt.plot(df['Force (N)'].values)
        for target in [df['IR (16 bit)'], df['Force (16 bit)']]:
            # dx = np.mean(np.diff(data0.x.values))
            shift = (np.argmax(signal.correlate(df['Force (N)'].values, target.values[1:3586])) - len(target.values[1:3586]))
            shifts.append(shift)
            # plt.plot(np.linspace(1,3586,3586) + shift, target.values[1:3587])

        # plt.show()
        fig, ax1 = plt.subplots()
        plt.Figure(figsize=(10,5))
        ax1.plot(df['Force (N)'], 'b-',linewidth=0.5)
        ax1.set_ylabel('Force (N)', color='b')
        ax1.tick_params('y', colors='b')

        ax2 = ax1.twinx()
        ax2.plot(np.linspace(1,3586,3586) + shifts[1], df['Force (16 bit)'].values[1:3587], 'r-',linewidth=0.5)
        ax2.set_ylabel('Force(16bit)', color='r')
        ax2.tick_params('y', colors='r')

        ax3 = ax1.twinx()
        # Offset the right spine of par2.  The ticks and label have already been
        # placed on the right by twinx above.
        ax3.spines["right"].set_position(("axes", 1.2))
        # Having been created by twinx, par2 has its frame off, so the line of its
        # detached spine is invisible.  First, activate the frame but make the patch
        # and spines invisible.
        make_patch_spines_invisible(ax3)
        # Second, show the right spine.
        ax3.spines["right"].set_visible(True)
        ax3.plot(np.linspace(1,3586,3586) + shifts[0], df['IR (16 bit)'].values[1:3587], 'g-',linewidth=0.5)
        ax3.set_ylabel('IR (16bit)', color='g')
        ax3.tick_params(axis='y', colors='g')

        plt.title('{}degree {}N maxF'.format(angle, maxForce))
        fig.tight_layout()
        plt.show()

# print (df_empty)

# lable_encoder = LabelEncoder().fit(df_empty.degree)
# lables = lable_encoder.transform(df_empty.degree)
# print (lable_encoder.classes_)
