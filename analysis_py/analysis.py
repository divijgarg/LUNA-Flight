import pandas as pd

import matplotlib.pyplot as plt


# Load the CSV file
def load_csv(file_path):
    try:
        data = pd.read_csv(file_path)
        return data
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


# Analyze and plot the data
def analyze_and_plot(file_path):
    data = load_csv(file_path)
    if data is None:
        return

    # Check if required columns exist
    # if "timestamp" not in data.columns or "kalman.position.px" not in data.columns:
    #     print("CSV file must contain 'timestamp' and 'kalman' columns.")
    #     return

    # Convert timestamp to datetime for better plotting
    # data['timestamp'] = pd.to_datetime(data['timestamp'])

    # Plot the data
    plt.figure()
    a = 100600
    plt.plot(
        (data["time"])/1000- 2303.840, (data["pos_x"]), label="Kalman Position X"
    )
    # plt.plot(
    #     (data["timestamp"])[:a]/1000- 2303.845, (data["kalman.velocity.vx"])[:a], label="Kalman Velocity X"
    # )
    # plt.plot(
    #     (data["timestamp"])[:a]/1000- 2303.845, (data["kalman.acceleration.ax"])[:a], label="Kalman Acceleration X"
    # )
    # plt.plot(
    #     (data["timestamp"])[:a]/1000- 2303.845, (data["orientation.angular_velocity.vx"])[:a], label="Angular Velocity X"
    # )
    # plt.plot(data["timestamp"][:a\])
    # plt.plot((data['timestamp']), (data['barometer.altitude']), label='Barometer')
    # plt.plot((data['timestamp'])[:a], (data['highg.ax'])[:a]*9.8, label='Accelerometer A_X')

    # plt.plot((data['timestamp']), (data['fsm']), label='Kalman v_X')
    # plt.plot((data['timestamp']), (data['kalman.velocity.vx']), label='Kalman v_X')
    # plt.plot(data['timestamp'], data['orientation.orientation_velocity.vx'], label='Orientation')
    plt.xlabel("Timestamp")
    plt.ylabel("Kalman Data")
    plt.title("Timestamp vs Kalman ")
    plt.legend()
    plt.grid()
    plt.show()


# Example usage
if __name__ == "__main__":
    file_path = "./data_files/kalman_output.csv"  # Replace with your CSV file path
    analyze_and_plot(file_path)
