import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_squared_error
import joblib
import threading

class MarkerDataCollector(Node):
    def __init__(self):
        super().__init__('marker_data_collector')
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'deproj_cones',
            self.marker_callback,
            10)
        self.steering_angle_subscriber = self.create_subscription(
            Float64,
            'steering_angle',
            self.steering_angle_callback,
            10)
        self.data = []
        self.current_steering_angle = None
        self.teaching_started = False

        self.input_thread = threading.Thread(target=self.check_for_training_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def marker_callback(self, msg):
        if self.current_steering_angle is None:
            self.get_logger().warn("Kormányszög hiányzik, várakozás az értékre...")
            return
        
        points = []
        for marker in msg.markers:
            points.append((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))
        
        self.data.append({
            'points': points,
            'turn_angle': self.current_steering_angle
        })
        self.get_logger().info(f"Adatok rögzítve: {len(self.data)} pont.")

    def steering_angle_callback(self, msg):
        self.current_steering_angle = msg.data
        self.get_logger().info(f"Új kormányszög érték beolvasva: {self.current_steering_angle}")

    def save_data(self):
        df = pd.DataFrame(self.data)
        df.to_csv('marker_data.csv', index=False)
        self.get_logger().info("Adatok elmentve marker_data.csv fájlba.")

    def train_model(self):
        if len(self.data) < 5:
            self.get_logger().error("Nem elegendő adat a modell tanításához!")
            return

        df = pd.DataFrame(self.data)
        df['x'] = df['points'].apply(lambda points: [p[0] for p in points])
        df['y'] = df['points'].apply(lambda points: [p[1] for p in points])
        
        df['avg_x'] = df['x'].apply(lambda x: sum(x) / len(x) if len(x) > 0 else 0)
        df['avg_y'] = df['y'].apply(lambda y: sum(y) / len(y) if len(y) > 0 else 0)
        
        X = df[['avg_x', 'avg_y']]
        y = df['turn_angle']

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=60)

        model = RandomForestRegressor(n_estimators=100, random_state=60)
        model.fit(X_train, y_train)

        predictions = model.predict(X_test)
        mse = mean_squared_error(y_test, predictions)
        self.get_logger().info(f"Tesztelési hiba (RMSE): {mse ** 0.5}")

        joblib.dump(model, 'turn_angle_model.pkl')
        self.get_logger().info("Modell elmentve turn_angle_model.pkl fájlba.")

    def check_for_training_input(self):
        while rclpy.ok():
            user_input = input("Nyomd meg az 'f' billentyűt a tanítás elindításához: ")
            if user_input.lower() == 'f':
                self.get_logger().info("Tanítás elindítása...")
                self.train_model()

def main(args=None):
    rclpy.init(args=args)
    collector = MarkerDataCollector()
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.save_data()
    finally:
        try:
            collector.destroy_node()
        except Exception as e:
            collector.get_logger().error(f"Hiba a node leállításakor: {e}")
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
