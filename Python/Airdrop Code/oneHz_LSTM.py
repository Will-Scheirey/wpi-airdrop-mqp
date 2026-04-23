import os

import tensorflow as tf
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib import gridspec
from sklearn.metrics import mean_squared_error
import re
from pathlib import Path

print("GPUs visible to TF:", tf.config.list_physical_devices("GPU"))

LOOKBACK = 25
STRIDE = 10
TRAIN_VAL_SPLIT = 0.2

epochs = 20     # duration of training
show_figs = 1
save_figs = 1
use_moving_avg_metrics = 1
use_all_full = 1            # whether to include accelerometer data or only roll + pitch
payload_cols = [], []
predicted_wspd, actual_wspd = [], []

base_loc = "/home/pmrust/MQP"
data_loc = base_loc + "/haars_data"

job_id = os.environ.get("SLURM_JOB_ID", "local")
plot_flight_dir = f"plots/flights/lstm_{job_id}"
plot_loss_dir = f"plots/loss/lstm_{job_id}"
os.makedirs(plot_flight_dir, exist_ok=True)
os.makedirs(plot_loss_dir, exist_ok=True)

if use_all_full:
    payload_cols  = ['alt_msl_m', 'wx_dps', 'wy_dps', 'a_h1_ms2', 'a_h2_ms2', 'a_v_ms2']
else:
    payload_cols = ['alt_msl_m', 'wx_dps', 'wy_dps']


def main():
    aug5_data = data_loc + "/08-05-2025"
    aug7_data = data_loc = "/08-07-2025"

    data_out = open(f'metrics/lstm_{job_id}.txt', 'a')

    aug5_flights = build_flights(aug5_data)
    aug7_flights = build_flights(aug7_data)

    aug5_flight_data = []
    for f in aug5_flights:
        X, y_wspd, y_wdir, _ = load_flight(f["sensor_path"], f["wind_path"], f["title"])
        aug5_flight_data.append({
            "title": f["title"],
            "X": X,
            "y_wspd": y_wspd,
            "y_wdir": y_wdir,
        })
    print(f"Loaded {len(aug5_flight_data)} flights from August 5, 2025")

    aug7_flight_data = []
    for f in aug7_flights:
        X, y_wspd, y_wdir, _ = load_flight(f["sensor_path"], f["wind_path"], f["title"])
        aug7_flight_data.append({
            "title": f["title"],
            "X": X,
            "y_wspd": y_wspd,
            "y_wdir": y_wdir,
        })
    print(f"Loaded {len(aug7_flight_data)} flights from August 7, 2025")

    for test_group in range(aug5_flight_data):
        test_data = aug5_flight_data[test_group]

        train_data = [flight_data[i] for i in range(NUM_FLIGHTS) if i != test_group]

        Title = f"TEST={test_data['title']} (train on {NUM_FLIGHTS} flights)"

        X_train_list = [data["X"] for data in train_data]
        X_all_list = [data["X"] for data in train_data] + [test_data["X"]]

        X_all_norm, mu, sigma = normalize_data(X_train_list, X_all_list)

        X_w_all, y_w_all = [], []
        for i, data in enumerate(train_data):
            data["X_norm"] = X_all_norm[i]
            test_data["X_norm"] = X_all_norm[-1]

            X_w, y_w = make_windows(data["X_norm"], data["y_wspd"], data["y_wdir"], lookback=LOOKBACK, stride=STRIDE)
            X_w_all.append(X_w)
            y_w_all.append(y_w)

        X_w = np.concatenate(X_w_all, axis=0)
        y_w = np.concatenate(y_w_all, axis=0)

        X_train, y_train, X_val, y_val = train_val_split(X_w, y_w, TRAIN_VAL_SPLIT)

        X_test, y_test = make_windows(test_data["X_norm"], test_data["y_wspd"], test_data["y_wdir"], lookback=LOOKBACK, stride=STRIDE)

        model = build_LSTM(numFeatures=X_train.shape[-1])
        train(model, epoch, X_train, y_train, X_val, y_val, Title)

        pred_test = model.predict(X_test, verbose=0).squeeze()
        pred_full = model.predict(X_w, verbose=0).squeeze()

        y_test = np.asarray(y_test).squeeze()
        y_w = np.asarray(y_w).squeeze()

        n = min(len(pred_test), len(y_test))
        pred_test = pred_test[:n]
        y_test = y_test[:n]

        m = min(len(pred_full), len(y_w))
        pred_full = pred_full[:m]
        y_w = y_w[:m]

        # MODEL ANALYSIS
        model_analysis(y_w, pred_full, Title, data_out)

        if show_figs:
            dt = 0.077
            fs = 1.0 / dt
            t_test_min = ((np.arange(n) * STRIDE + (LOOKBACK - 1)) * dt) / 60.0
            plt.figure(figsize=(10, 3))

            plt.plot(t_test_min, y_test,
                    label="True wind speed",
                    linewidth=1.5)

            plt.plot(t_test_min, pred_test,
                    label="Predicted wind speed",
                    linewidth=1.5,
                    alpha=0.9)

            plt.xlabel("Time (minutes)")
            plt.ylabel("Wind speed (m/s)")
            plt.title(f"Test flight: {Title}")
            plt.legend()
            plt.grid(True, linestyle="--", alpha=0.5)

            plt.tight_layout()
            safe_title = re.sub(r"[^A-Za-z0-9._-]+", "_", Title)
            plt.savefig(os.path.join(plot_flight_dir, f"{safe_title}.png"), dpi=300)
            plt.close()
    data_out.close()

def plot_single_moving_avg(t1, ground_truth, labels, y_label, y_lims, title, show_metrics):
    window_length = 10

    t1_avg = np.convolve(t1, np.ones((window_length,)) / window_length, mode='valid')
    gt_avg = np.convolve(ground_truth, np.ones((window_length,)) / window_length, mode='valid')
    lf = min(len(t1_avg), len(gt_avg))
    t1_avg = t1_avg[:lf]
    gt_avg = gt_avg[:lf]

    fig = plt.figure(figsize=(8.75, 3.25))
    gs = gridspec.GridSpec(1, 1, height_ratios=[1])
    ax0 = plt.subplot(gs[0])
    fig.tight_layout()
    axes = [ax0]
    a = plt.gca()
    axes[0].set_ylim(ymin=0, ymax=6)
    axes[0].set_xlim([-0.25, 16])

    t1_min = np.divide(range(len(t1_avg)), 60)
    gt_min = np.divide(range(len(gt_avg)), 60)

    colors = ['#232D4B', '#E57200']

    axes[0].plot(gt_min, gt_avg, label=labels[1], color=colors[1], alpha=0.75, linewidth=1.82)
    axes[0].plot(t1_min, t1_avg, label=labels[0], color=colors[0], alpha=0.8, linewidth=1.82)

    # metrics
    drone_metrics = [MBE(gt_avg, t1_avg), np.sqrt(mean_squared_error(gt_avg, t1_avg))]

    if show_metrics:
        metric_label = ('Payload RMSE:   %.02f m s$^{-1}$  Payload MBE:   %.02f m s$^{-1}$' %
                        (drone_metrics[1], drone_metrics[0]))
        axes[0].annotate(
            metric_label,
            (0.01, 0.965),
            xytext=(4, -4),
            xycoords='axes fraction',
            textcoords='offset points',
            fontweight='bold',
            color='k',
            backgroundcolor='white',
            ha='left', va='top'
        )

    axes[0].set_ylabel(y_label, fontsize=18)
    ax0.set_xlabel("Time (minutes)", fontsize=18)
    axes[0].legend(loc="upper right", ncol=1, frameon=True, prop={'size': 13})
    ax0.yaxis.set_ticks(np.arange(0, y_lims[1], 1))
    ax0.tick_params(axis='both', which='major', labelsize=12)
    plt.subplots_adjust(top=0.885, bottom=0.194, left=0.1, right=0.94, hspace=0.2, wspace=0.06)

    if save_figs:
        plt.savefig(base_loc + "\\single_trial_%s.png" % (title))

    plt.grid(linestyle="--")
    if show_figs:
        plt.show()

    plt.close()

def plot_mult_moving_avg(t1, t2, ground_truth, labels, y_label, y_lims, title, show_metrics):
    window_length = 10

    t1_avg = np.convolve(t1, np.ones((window_length,)) / window_length, mode='valid')
    t2_avg = np.convolve(t2, np.ones((window_length,)) / window_length, mode='valid')
    gt_avg = np.convolve(ground_truth, np.ones((window_length,)) / window_length, mode='valid')
    lf = min(len(t1_avg), len(t2_avg), len(gt_avg))
    t1_avg = t1_avg[:lf]
    t2_avg = t2_avg[:lf]
    gt_avg = gt_avg[:lf]

    fig = plt.figure(figsize=(8.75, 3.25))
    gs = gridspec.GridSpec(1, 1, height_ratios=[1])
    ax0 = plt.subplot(gs[0])
    fig.tight_layout()
    axes = [ax0]
    a = plt.gca()
    axes[0].set_ylim(ymin=0, ymax=6)
    axes[0].set_xlim([-0.25, 16])

    t1_min = np.divide(range(len(t1_avg)), 60)
    t2_min = np.divide(range(len(t2_avg)), 60)
    gt_min = np.divide(range(len(gt_avg)), 60)

    colors = ['#232D4B', '#64a644', '#E57200']

    axes[0].plot(gt_min, gt_avg, label=labels[2], color=colors[2], alpha=0.75, linewidth=1.82)
    axes[0].plot(t1_min, t1_avg, label=labels[0], color=colors[0], alpha=0.8, linewidth=1.82)
    axes[0].plot(t2_min, t2_avg, label=labels[1], color=colors[1], alpha=0.8, linewidth=1.82)

    # metrics
    solo_metrics = [MBE(gt_avg, t1_avg), np.sqrt(mean_squared_error(gt_avg, t1_avg))]
    mav_metrics = [MBE(gt_avg, t2_avg), np.sqrt(mean_squared_error(gt_avg, t2_avg))]

    if show_metrics:
        metric_label = ('Solo RMSE:   %.02f m s$^{-1}$  Solo MBE:   %.02f m s$^{-1}$\n'
                        'Mavic RMSE: %.02f m s$^{-1}$ Mavic MBE: %.02f m s$^{-1}$' %
                        (solo_metrics[1], solo_metrics[0], mav_metrics[1], mav_metrics[0]))
        axes[0].annotate(
            metric_label,
            (0.01, 0.965),
            xytext=(4, -4),
            xycoords='axes fraction',
            textcoords='offset points',
            fontweight='bold',
            color='k',
            backgroundcolor='white',
            ha='left', va='top'
        )

    axes[0].set_ylabel(y_label, fontsize=18)
    ax0.set_xlabel("Time (minutes)", fontsize=18)
    axes[0].legend(loc="upper right", ncol=1, frameon=True, prop={'size': 13})
    ax0.yaxis.set_ticks(np.arange(0, y_lims[1], 1))
    ax0.tick_params(axis='both', which='major', labelsize=12)
    plt.subplots_adjust(top=0.885, bottom=0.194, left=0.1, right=0.94, hspace=0.2, wspace=0.06)

    if save_figs:
        plt.savefig(base_loc + "\\multi_trial_%s.png" % (title))

    plt.grid(linestyle="--")
    if show_figs:
        plt.show()

    plt.close()

def build_flights(output_folder):
    out = Path(output_folder)

    # Index by base_name
    imu = {}
    wind = {}

    def base_from(prefix, path: Path):
        m = re.match(rf"{re.escape(prefix)}_(.*)\.csv$", path.name)
        return m.group(1) if m else None

    for p in out.glob("imu_*.csv"):
        b = base_from("imu", p)
        if b: imu[b] = p

    for p in out.glob("wind_*.csv"):
        b = base_from("wind", p)
        if b: wind[b] = p

    bases = sorted(set(imu) & set(wind))

    flights = []
    for b in bases:
        flights.append({
            "title": b,
            "sensor_path": str(imu[b]),
            "wind_path": str(wind[b]),
        })

    return flights


def load_flight(sensor_csv, wind_csv, title):
    X = np.array(pd.read_csv(sensor_csv)[payload_cols])
    y_wspd = np.array(pd.read_csv(wind_csv)['wind_speed_mps'])
    y_wdir = np.array(pd.read_csv(wind_csv)['wind_dir_deg'])

    n = min(len(X), len(y_wspd), len(y_wdir))
    return X[:n], y_wspd[:n], y_wdir[:n], title

'''
Standard Scaling:
Scales data columns by taking each row, subtracting the column mean from it,
and dividing it by the column standard deviation
'''
def normalize_data(X_train, X_total):
    X_cat = np.concatenate(X_train, axis=0)
    mu = np.mean(X_cat, axis=0)
    sigma = np.std(X_cat, axis=0)

    X_tfm = []
    for i in X_total:
        X_tfm.append((i - mu)/sigma)

    return X_tfm, mu, sigma

def make_windows(X, y, lookback, stride):
    X_w, y_w = [], []
    T = min(len(X), len(y))
    X = X[:T]
    y = y[:T]

    for t in range(lookback, T, stride):
        X_w.append(X[t - lookback:t])
        y_w.append(y[t - 1])

    return np.asarray(X_w), np.asarray(y_w).reshape(-1, 1)

def train_val_split(X, y, fraction):
    idx = np.random.permutation(len(X))

    split = int((1-fraction)*len(idx))
    train_idx, val_idx = idx[:split], idx[split:]

    return X[train_idx], y[train_idx], X[val_idx], y[val_idx]

def build_LSTM(numFeatures):
    inp = tf.keras.Input(shape=(LOOKBACK, numFeatures))  # (timesteps, features)
    x = tf.keras.layers.Conv1D(filters=32, kernel_size=5, padding="same", activation="relu")(inp)
    x = tf.keras.layers.LSTM(64, kernel_regularizer=tf.keras.regularizers.l1(1e-3), return_sequences=True)(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    x = tf.keras.layers.LSTM(32, kernel_regularizer=tf.keras.regularizers.l1(1e-3), return_sequences=False)(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    x = tf.keras.layers.Dense(32, kernel_regularizer=tf.keras.regularizers.l1(1e-3), activation="relu")(x)
    out = tf.keras.layers.Dense(2)(x)

    model = tf.keras.Model(inp, out)
    optimizer = tf.keras.optimizers.RMSprop(learning_rate=1e-3)

    # TF-native RMSE metric
    def rmse_tf(y_true, y_pred):
        return tf.sqrt(tf.reduce_mean(tf.square(y_pred - y_true)))

    model.compile(optimizer=optimizer, loss='mse', metrics=[rmse_tf])
    return model

def model_LSTM(numFeatures):

    # march model 420-180-90d92-42-lr4e-5
    inp = tf.keras.Input(shape=(LOOKBACK, numFeatures))
    x = tf.keras.layers.LSTM(420, return_sequences=True)(inp)
    x = tf.keras.layers.LSTM(180, return_sequences=True)(x)
    x = tf.keras.layers.LSTM(90, return_sequences=True, dropout=0.96)(x) # dropout used to combat overfitting in one flight during June
    #multi_step_model.add(tf.keras.layers.LSTM(90, return_sequences=True))
    x = tf.keras.layers.LSTM(48, activation='relu')(x)


    #multi_step_model.add(LeakyReLU(alpha=0.05))activation=tf.nn.tanh)
    out = tf.keras.layers.Dense(2)(x)
    model = tf.keras.Model(inp, out)
    optimizer = tf.keras.optimizers.RMSprop(learning_rate=4e-5)
    
    def rmse_tf(y_true, y_pred):
        return tf.sqrt(tf.reduce_mean(tf.square(y_pred - y_true)))

    model.compile(optimizer=optimizer, loss = 'mse')

    return model

# kernel_regularizer=tf.keras.regularizers.l1(1e-3)
'''
Trains the model on given data
'''
def train(multi_step_model, epoch, trainData, trainLabels, testData, testLabels, Title):
    callback = tf.keras.callbacks.EarlyStopping(monitor='loss', patience=10)
    t = multi_step_model.fit(trainData, trainLabels, epochs=epoch, batch_size=16,
                             validation_data=(testData, testLabels), callbacks=[callback])
    plt.plot(t.history['loss'])
    plt.plot(t.history['val_loss'])
    plt.title(Title + ' train vs validation loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.legend(['train', 'validation'], loc='upper right', ncol=2)
    if show_figs:
        safe_title = re.sub(r"[^A-Za-z0-9._-]+", "_", Title)
        plt.savefig(os.path.join(plot_loss_dir, f"{safe_title}_FULL_Y.png"), dpi=300)
    plt.close()


def model_analysis(y_test, y_pred, Title, data_out):
    # TrueWspdVar, PredWspdVar, MBE, MSE, RMSE
    metrics = [
        np.std(y_test) ** 2,
        np.std(y_pred) ** 2,
        MBE(y_test, y_pred),
        mean_squared_error(y_test, y_pred),
        np.sqrt(mean_squared_error(y_test, y_pred))
    ]

    data_out.write(f"{Title} \n")
    data_out.write(f"{'true wind speed variance':34s}: {metrics[0]:4.2f}\n")
    data_out.write(f"{'predicted wind speed variance':34s}: {metrics[1]:4.2f}\n")
    data_out.write(f"{'MBE':34s}: {metrics[2]:4.2f}\n")
    data_out.write(f"{'MSE':34s}: {metrics[3]:4.2f}\n")
    data_out.write(f"{'RMSE':34s}: {metrics[4]:4.2f}\n")
    data_out.write(f"{'true turbulence intensity':34s}: {findTurbulence(y_test):4.2f}\n")
    data_out.write(f"{'predicted turbulence intensity':34s}: {findTurbulence(y_pred):4.2f}\n")
    data_out.write(f"{'turbulence intensity diff':34s}: {findTurbulence(y_test) - findTurbulence(y_pred):4.2f}\n")

'''
Plots the predictions for the given model on the given data
'''
def plot_predictions(multi_step_model, testData, trainLabels, testLabels, Title, training_increment, save_fig):
    for i in range(1):
        fig = plt.figure(figsize=(12, 3))
        gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1])
        ax0 = plt.subplot(gs[0])
        ax1 = plt.subplot(gs[1])

        axes = [ax0, ax1]

        x, y = testData, testLabels
        predictions = multi_step_model.predict(x, verbose=0)[:, 0]
        avg = np.mean(predictions)
        flipped_predictions = []
        for p in predictions:
            flipped_predictions.append(avg + (avg - p))
        # predictions = flipped_predictions  # keep commented unless you want the flip

        plt.figure(i)
        a = plt.gca()
        a.set_ylim([0, 9])

        bf = len(trainLabels)
        td = len(testData)

        axes[0].plot(range(bf), trainLabels, label='History', color='Orange')
        axes[0].plot(range(bf, bf + td), y, label='True Wspd', color='#c334e3', alpha=0.8)
        axes[0].plot(range(bf, bf + td), predictions, label='Predicted Wspd', color='Navy')

        axes[1].plot(range(bf, bf + td), y, label='True Wspd', color='#c334e3', alpha=0.8, linewidth=1.1)
        axes[1].plot(range(bf, bf + td), predictions, label='Predicted Wspd', color='Navy', linewidth=0.8)

        fig.tight_layout()

        # Set labels
        fig.text(0.5, 0.04, 'Time Step', ha='center', va='center')
        fig.text(0, 0.8, 'Wspd', ha='center', va='center', rotation='vertical')

        axes[0].set_title(Title + " - Training + Testing Data (%1f split)" % (training_increment * 10))
        axes[1].set_title(Title + " - Testing Data Only")

        ax0.legend(loc="best")
        ax1.legend(loc="best")
        plt.legend(ncol=2)
        plt.subplots_adjust(top=0.885, bottom=0.194, left=0.06, right=0.94, hspace=0.2, wspace=0.06)

        try:
            moving_metrics = plot_moving_avg_final(np.array(y).ravel(), predictions, Title, training_increment)
        except Exception as e:
            print(e)
            moving_metrics = [metrics[2], metrics[4]]  # fallback to MBE, RMSE if needed

        plt.close()
        if use_moving_avg_metrics:
            return moving_metrics
        return metrics


def findTurbulence(ws):
    return round(np.std(ws) / np.mean(ws), 3)


def MBE(y_true, y_pred):
    '''
    Parameters:
        y_true (array): Array of observed values
        y_pred (array): Array of prediction values
    Returns:
        mbe (float): bias score
    '''
    y_true = np.array(y_true).reshape(-1, 1)
    y_pred = np.array(y_pred).reshape(-1, 1)
    diff = (y_pred - y_true)
    mbe = diff.mean()
    return round(mbe, 3)


'''
1D turbulence utility functions
'''
def find_turb_var(u, v):
    u_var = np.var(u)
    v_var = np.var(v)
    print("u turb var: %.03f\nv turb var: %.03f" % (u_var, v_var))
    return u_var, v_var

# convert wind to streamwise coordinates
# (https://www.eol.ucar.edu/content/wind-direction-quick-reference)
def to_streamwise_coord(u, v, wdir):
    u_stream = []
    v_stream = []
    mean_wind_dir = np.mean(wdir)
    for i in range(len(u)):
        d = np.degrees(np.arctan2(v[i], u[i]))
        u_stream.append(u[i] * np.cos(np.radians(d)) + v[i] * np.sin(np.radians(d)))
        v_stream.append(-1 * u[i] * np.sin(np.radians(d)) + v[i] * np.cos(np.radians(d)))
    return u_stream, v_stream

# http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
def split_wspd(wspd, wdir):
    u = []
    v = []
    avg_wind_dir = np.mean(wdir)
    for w in range(len(wspd)):
        met_dir = avg_wind_dir + 180
        u.append(-1 * wspd[w] * np.sin((np.pi / 180) * met_dir))
        v.append(-1 * wspd[w] * np.cos((np.pi / 180) * met_dir))
    return u, v

if __name__ == "__main__":
    main()

