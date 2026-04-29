import os
import tensorflow as tf
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
import re
from pathlib import Path

#------------GLOBAL VARIABLES------------
LOOKBACK = 25
STRIDE = 10
TRAIN_VAL_SPLIT = 0.2
SAMPLING_TIME = 0.077

epochs = 20         # duration of training
save_figs = 1       # whether or not to save figure outputs
input_data = 1      # data columns to be included in input
payload_cols = []

if input_data == 1:
    payload_cols  = ['alt_msl_m', 'wx_dps', 'wy_dps', 'a_h1_ms2', 'a_h2_ms2', 'a_v_ms2']
elif input_data == 2:
    payload_cols = ['alt_msl_m', 'wx_dps', 'wy_dps']
elif input_data == 3:
    payload_cols = ['alt_msl_m', 'a_h1_ms2', 'a_h2_ms2', 'a_v_ms2']
elif input_data == 4:
    payload_cols = ['wx_dps', 'wy_dps', 'a_h1_ms2', 'a_h2_ms2', 'a_v_ms2']
elif input_data == 5:
    payload_cols = ['wx_dps', 'wy_dps']
elif input_data == 6:
    payload_cols = ['a_h1_ms2', 'a_h2_ms2', 'a_v_ms2']
else:
    payload_cols = ['alt_msl_m']

#------------FILE PATHS------------
job_id = os.environ.get("SLURM_JOB_ID", "local")
plot_flight_dir = f"plots/flights/lstm_{job_id}"
plot_loss_dir = f"plots/loss/lstm_{job_id}"
os.makedirs(plot_flight_dir, exist_ok=True)
os.makedirs(plot_loss_dir, exist_ok=True)

def main():
    base_loc = "/path/to/data_input"

    # Load and store train and test flights
    train_data = base_loc + "/path/to/train_data"
    test_data = base_loc + "/path/to/test_data"

    data_out = open(f'metrics/lstm_{job_id}_{input_data}.txt', 'a')

    train_flights = build_flights(train_data)
    test_flights = build_flights(test_data)

    train_flight_data = []
    for f in train_flights:
        X, y_wspd, y_wdir, _ = load_flight(f["sensor_path"], f["wind_path"], f["title"])
        train_flight_data.append({
            "title": f["title"],
            "X": X,
            "y_wspd": y_wspd,
            "y_wdir": y_wdir
        })          
    print(f"Loaded {len(train_flight_data)} flights from training dataset")

    test_flight_data = []
    for f in test_flights:
        X, y_wspd, y_wdir, _ = load_flight(f["sensor_path"], f["wind_path"], f["title"])
        test_flight_data.append({
            "title": f["title"],
            "X": X,
            "y_wspd": y_wspd,
            "y_wdir": y_wdir
        })
    print(f"Loaded {len(test_flight_data)} flights from testing dataset")

    # Initialize arrays for storing all model outputs for graphing
    all_wspd_true = []
    all_wspd_pred = []
    all_wspd_err = []
    mbe_store, rmse_store = [], []

    fig_wspd, ax1 = plt.subplots(figsize=(10,3))
    fig_werr, ax2 = plt.subplots(figsize=(10,3))

    # Leave-One-Out Cross-Validation on each flight in test dataset
    for test_group in range(len(train_flight_data)):
        test_data = train_flight_data[test_group]
        train_data = test_flight_data

        Title = f"TEST={test_data['title']}"
        X_train_list = [data["X"] for data in train_data]
        X_all_list = [data["X"] for data in train_data] + [test_data["X"]]

        # Normalize train and test datasets based on train dataset statistics
        X_all_norm, mu, sigma = normalize_data(X_train_list, X_all_list)

        # Create lookback windows out of train dataset, converting environmental wind speed to its orthogonal components
        X_w_all, y_w_all = [], []
        test_data["X_norm"] = X_all_norm[-1]
        for i, data in enumerate(train_data):
            data["X_norm"] = X_all_norm[i]
            wdir_rad = np.deg2rad(data["y_wdir"])
            y_train = np.asarray([data["y_wspd"], np.sin(wdir_rad), np.cos(wdir_rad)]).T

            X_w, y_w = make_windows(data["X_norm"], y_train, lookback=LOOKBACK, stride=STRIDE)
            X_w_all.append(X_w)
            y_w_all.append(y_w)
        X_w = np.concatenate(X_w_all, axis=0)
        y_w = np.concatenate(y_w_all, axis=0)
        # Split training dataset into train and validation windows
        X_train, y_train, X_val, y_val = train_val_split(X_w, y_w, TRAIN_VAL_SPLIT)

        # Create lookback windows out of test dataset, converting environmental wind speed to its orthogonal components
        wdir_rad = np.deg2rad(test_data["y_wdir"])
        y_test = np.asarray([test_data["y_wspd"], np.sin(wdir_rad), np.cos(wdir_rad)]).T
        X_test, y_test = make_windows(test_data["X_norm"], y_test, lookback=LOOKBACK, stride=STRIDE)

        # Build and train LSTM model
        train_labels = {
            "speed": y_train[:, 0],
            "dir": y_train[:, 1:3]
        }
        val_labels = {
            "speed": y_val[:, 0],
            "dir": y_val[:, 1:3]
        }
        model = build_LSTM(numFeatures=X_train.shape[-1])
        train(model, epochs, X_train, train_labels, X_val, val_labels, Title)

        # Predict labels for test flight
        pred_speed, pred_dir = model.predict(X_test, verbose=0)
        pred_speed = np.asarray(pred_speed)
        pred_dir = np.asarray(pred_dir)
        y_test = np.asarray(y_test)

        # Store true and predicted windspeeds for graphing and analysis
        plot_len = min(len(pred_speed), len(y_test))

        wspd_true = y_test[:plot_len, 0]
        wspd_pred = pred_speed[:plot_len, 0]
        wspd_err  = wspd_true - wspd_pred

        all_wspd_true.append(wspd_true)
        all_wspd_pred.append(wspd_pred)
        all_wspd_err.append(wspd_err)

        # Store Mean Bias Error and Root Mean Square Error across test flight
        mbe_store.append(MBE(wspd_true, wspd_pred))
        rmse_store.append(RMSE(wspd_true, wspd_pred))

    # Trim all datasets to shortest flight duration
    global_plot_len = min(len(x) for x in all_wspd_true)

    all_wspd_true = np.stack([x[:global_plot_len] for x in all_wspd_true], axis=0)
    all_wspd_pred = np.stack([x[:global_plot_len] for x in all_wspd_pred], axis=0)
    all_wspd_err  = np.stack([x[:global_plot_len] for x in all_wspd_err], axis=0)

    # Calculate mean of all datasets
    true_mean = np.mean(all_wspd_true, axis=0)
    pred_mean = np.mean(all_wspd_pred, axis=0)
    err_mean  = np.mean(all_wspd_err, axis=0)

    # Calculate 1 standard deviation of all datasets
    true_std = np.std(all_wspd_true, axis=0)
    pred_std = np.std(all_wspd_pred, axis=0)
    err_std  = np.std(all_wspd_err, axis=0)

    fig_wspd, ax1 = plt.subplots(figsize=(10, 3))

    # Create time axis based on input data sample rate
    SAMPLING_TIME = 0.077   # seconds
    t_test_min = ((np.arange(global_plot_len) * STRIDE + (LOOKBACK - 1)) * dt) / 60.0

    # Plot true and predicted environmental wind speeds
    ax1.plot(t_test_min, true_mean, label="True Wind Speed", linewidth=2)
    ax1.plot(t_test_min, pred_mean, label="Predicted Wind Speed", linewidth=2)

    ax1.fill_between(t_test_min, true_mean - true_std, true_mean + true_std, alpha=0.2)
    ax1.fill_between(t_test_min, pred_mean - pred_std, pred_mean + pred_std, alpha=0.2)

    ax1.set_xlabel("Time (min)")
    ax1.set_ylabel("Wind Speed (m/s)")
    ax1.set_title(f"Mean Wind Speed Across {len(train_flight_data)} Flights")
    ax1.legend()
    ax1.grid(True, linestyle="--", alpha=0.5)

    fig_wspd.tight_layout()
    fig_wspd.savefig(os.path.join(plot_flight_dir, "wspd_mean_band.png"), dpi=300)
    plt.close(fig_wspd)

    # Plot error between true and predicted environmental wind speeds
    fig_werr, ax2 = plt.subplots(figsize=(10, 3))

    ax2.plot(t_test_min, err_mean, label="True - Predicted Wind Speed", linewidth=2)
    ax2.fill_between(t_test_min, err_mean - err_std, err_mean + err_std, alpha=0.2)

    ax2.axhline(0, linewidth=1)
    ax2.set_xlabel("Time (min)")
    ax2.set_ylabel("Error (m/s)")
    ax2.set_title(f"Mean Wind Speed Error Across {len(train_flight_data)} Flights")
    ax2.legend()
    ax2.grid(True, linestyle="--", alpha=0.5)

    fig_werr.tight_layout()
    fig_werr.savefig(os.path.join(plot_flight_dir, "wspd_error_mean_band.png"), dpi=300)
    plt.close(fig_werr)

    # Print MBE and RMSE across all datasets--used to compare model/input data performance
    print(f"Mean Bias Error: {sum(mbe_store) / len(mbe_store)}")
    print(f"Root Mean Square Error: {sum(rmse_store) / len(rmse_store)}")

    data_out.close()


#----------FUNCTION DECLARATIONS----------
#***************************************************************************************************
# BUILD_FLIGHTS Loads and stores file paths for input payload states and environmental winds for each flight
#
# INPUTS:
#    output_folder  :   Path to input flight data CSVs
# OUTPUTS:
#    flights  :   Data matrix with specific wind/IMU paths for each flight
#***************************************************************************************************
def build_flights(output_folder):
    out = Path(output_folder)
    imu = {}
    wind = {}

    # Load files given specific file titles--subject to change per system
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

    # Concatenate data paths for each flight
    flights = []
    for b in bases:
        flights.append({
            "title": b,
            "sensor_path": str(imu[b]),
            "wind_path": str(wind[b]),
        })

    return flights

#***************************************************************************************************
# BUILD_FLIGHTS Loads and stores data from input payload states and environmental winds
#
# INPUTS:
#    sensor_csv  :   Path to payload IMU/state data
#    wind_csv    :   Path to environmental wind data
#    title       :   Flight label
# OUTPUTS:
#    flights  :   Data matrix with wind speeds and payload states for each flight loaded
#***************************************************************************************************
def load_flight(sensor_csv, wind_csv, title):
    # Load data given specific column labels--subject to change per system
    X = np.array(pd.read_csv(sensor_csv)[payload_cols])
    y_wspd = np.array(pd.read_csv(wind_csv)['wind_speed_mps'])
    y_wdir = np.array(pd.read_csv(wind_csv)['wind_dir_deg'])

    # Trim all columns to the shortest column length to maintain synchronization
    n = min(len(X), len(y_wspd), len(y_wdir))
    return X[:n], y_wspd[:n], y_wdir[:n], title


#***************************************************************************************************
# NORMALIZE_DATA Scales data columns by taking each row, subtracting the column mean from it, and dividing it by the column standard deviation
#
# INPUTS:
#   X_train :   Input data for train dataset
#   X_total :   Input data for both train and test datasets
#
# OUTPUTS:
#   X_tfm   :   Transformed and normalized dataset
#   mu      :   Dataset mean
#   sigma   :   Dataset standard deviation
#***************************************************************************************************
def normalize_data(X_train, X_total):
    X_cat = np.concatenate(X_train, axis=0)
    mu = np.mean(X_cat, axis=0)
    sigma = np.std(X_cat, axis=0)

    X_tfm = []
    for i in X_total:
        X_tfm.append((i - mu)/sigma)

    return X_tfm, mu, sigma

#***************************************************************************************************
# MAKE_WINDOWS Creates lookback windows for dataset given desired window length and stride
#
# INPUTS:
#   X           :   Input data
#   y           :   Data labels
#   lookback    :   Window length
#   stride      :   Samples skipped between windows
# OUTPUTS:
#   X_w         :   Windowed input data
#   y_w         :   Windowed data labels
#***************************************************************************************************
def make_windows(X, y, lookback, stride):
    X_w, y_w = [], []
    T = min(len(X), len(y))
    X = X[:T]
    y = y[:T]

    for t in range(lookback, T, stride):
        X_w.append(X[t - lookback:t])
        y_w.append(y[t - 1])

    return np.asarray(X_w), np.asarray(y_w)#.reshape(-1, 1)

#***************************************************************************************************
# TRAIN_VAL_SPLIT Reserves a portion of training dataset for validation given desired split ratio
#
# INPUTS:
#   X           :   Train dataset input
#   y           :   Train dataset labels
#   fraction    :   Desired data split between training and validation
# OUTPUTS:
#   X_train     :   Train dataset input
#   y_train     :   Train dataset labels
#   X_val       :   Validation dataset input
#   y_val       :   Validation dataset labels
#***************************************************************************************************
def train_val_split(X, y, fraction):
    # Location to begin data split is randomized
    idx = np.random.permutation(len(X))

    split = int((1-fraction)*len(idx))
    train_idx, val_idx = idx[:split], idx[split:]

    return X[train_idx], y[train_idx], X[val_idx], y[val_idx]

#***************************************************************************************************
# BUILD_LSTM Builds LSTM model for environmental wind estimation
#
# INPUTS:
#   numFeatures : Number of features based on input data
# OUTPUTS:
#   model       : Model object
#***************************************************************************************************
def build_LSTM(numFeatures):
    inp = tf.keras.Input(shape=(LOOKBACK, numFeatures))
    x = tf.keras.layers.Conv1D(filters=16, kernel_size=5, padding="same", activation="relu")(inp)
    x = tf.keras.layers.LSTM(64, 
        kernel_regularizer=tf.keras.regularizers.l1(1e-3),
        return_sequences=True)(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    x = tf.keras.layers.LSTM(32, 
        kernel_regularizer=tf.keras.regularizers.l1(1e-3), 
        return_sequences=False)(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    x = tf.keras.layers.Dense(32, 
        kernel_regularizer=tf.keras.regularizers.l1(1e-3), 
        activation="relu")(x)

    out_speed = tf.keras.layers.Dense(1, name="speed")(x)
    out_dir = tf.keras.layers.Dense(2, activation="tanh", name="dir")(x)
    model = tf.keras.Model(inp, [out_speed, out_dir])
    optimizer = tf.keras.optimizers.RMSprop(learning_rate=1e-3)

    # Loss is computed using Mean Bias Error
    model.compile(
        optimizer=optimizer,
        loss={"speed": "mse", "dir": "mse"},
    )
    return model


#***************************************************************************************************
# TRAIN Trains environmental wind estimation model
#
# INPUTS:
#   multi_step_model    :   Model object
#   epoch               :   Number of training epochs
#   trainData           :   Train dataset input
#   trainLabels         :   Train dataset labels
#   testLabels          :   Test dataset labels
#   Title               :   Flight label
# OUTPUTS:
#   plot                :   Training loss plot
#***************************************************************************************************
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
    if save_figs:
        safe_title = re.sub(r"[^A-Za-z0-9._-]+", "_", Title)
        plt.savefig(os.path.join(plot_loss_dir, f"{safe_title}_FULL_Y.png"), dpi=300)
    plt.close()

#***************************************************************************************************
# MBE Calculates Mean Bias Error of true versus predicted environmental winds
#
# INPUTS:
#   y_true  :   Ground truth environmental winds
#   y_pred  :   Predicted environmental winds
# OUTPUTS:
#   mbe     :   Bias score
#***************************************************************************************************
def MBE(y_true, y_pred):
    y_true = np.array(y_true).reshape(-1, 1)
    y_pred = np.array(y_pred).reshape(-1, 1)
    diff = (y_true - y_pred)
    mbe = diff.mean()
    return round(mbe, 3)

#***************************************************************************************************
# RMSE Calculates Root Mean Square Error of true versus predicted environmental winds
#
# INPUTS:
#   y_true  :   Ground truth environmental winds
#   y_pred  :   Predicted environmental winds
# OUTPUTS:
#   rmse    :   Error score
#***************************************************************************************************
def RMSE(y_true, y_pred):
    y_true = np.array(y_true).reshape(-1, 1)
    y_pred = np.array(y_pred).reshape(-1, 1)
    diff = (y_true - y_pred) **2
    sqrt = np.sqrt(diff)
    rmse = sqrt.mean()
    return round(rmse, 3)

if __name__ == "__main__":
    main()
