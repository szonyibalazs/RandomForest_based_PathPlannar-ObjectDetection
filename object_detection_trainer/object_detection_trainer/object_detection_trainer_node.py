import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
import joblib
import threading
import math

class MarkerDataCollector(Node):
    def __init__(self):
        super().__init__('marker_data_collector')
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'pc_markers',
            self.marker_callback,
            10)
        self.valid_blue_subscriber = self.create_subscription(
            MarkerArray,
            'Valid_blue',
            self.valid_blue_callback,
            10)
        self.valid_yellow_subscriber = self.create_subscription(
            MarkerArray,
            'Valid_yellow',
            self.valid_yellow_callback,
            10)
        self.data = []
        self.blue_points = []
        self.yellow_points = []
        self.teaching_started = False
        self.data_lock = threading.Lock()
        self.input_thread = threading.Thread(target=self.check_for_training_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def marker_callback(self, msg):
        points = [(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) for marker in msg.markers]
        with self.data_lock:
            for point in points:
                self.data.append({
                    'x': point[0],
                    'y': point[1],
                    'z': point[2],
                    'label': None
                })
        self.get_logger().info(f"Adatok rögzítve: {len(points)} pont.")

    def valid_blue_callback(self, msg):
        with self.data_lock:
            for marker in msg.markers:
                self.blue_points.append((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))

    def valid_yellow_callback(self, msg):
        with self.data_lock:
            for marker in msg.markers:
                self.yellow_points.append((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))

    def save_data(self):
        with self.data_lock:
            df = pd.DataFrame(self.data)
            df.to_csv('marker_data05.csv', index=False)
        self.get_logger().info("Adatok elmentve marker_data05.csv fájlba.")

    def is_within_radius(self, point1, point2, radius=0.5):
        distance = math.sqrt((point1[0] - point2[0]) ** 2 +
                             (point1[1] - point2[1]) ** 2 +
                             (point1[2] - point2[2]) ** 2)
        return distance <= radius

    def label_points(self):
        labeled_data = []
        with self.data_lock:
            for point in self.data:
                point_coordinates = (point['x'], point['y'], point['z'])
                for blue_point in self.blue_points:
                    if self.is_within_radius(point_coordinates, blue_point):
                        point['label'] = 'blue'
                        break
                if point['label'] is None:
                    for yellow_point in self.yellow_points:
                        if self.is_within_radius(point_coordinates, yellow_point):
                            point['label'] = 'yellow'
                            break
                if point['label'] is not None:
                    labeled_data.append(point)

        return labeled_data

    def train_model(self):
        with self.data_lock:
            if len(self.data) < 5:
                self.get_logger().error("Nem elegendő adat a modell tanításához!")
                return
            labeled_data = self.label_points()
            if len(labeled_data) == 0:
                self.get_logger().error("Nincs elegendő címkézett adat a modell tanításához!")
                return
            df = pd.DataFrame(labeled_data)

        df = df.dropna()
        X = df[['x', 'y', 'z']]
        y = df['label']
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=70)
        model = RandomForestClassifier(n_estimators=100, random_state=70)
        model.fit(X_train, y_train)
        predictions = model.predict(X_test)
        accuracy = accuracy_score(y_test, predictions)
        self.get_logger().info(f"Tesztelési pontosság: {accuracy * 100:.2f}%")
        joblib.dump(model, 'marker_classification_model05.pkl')
        self.get_logger().info("Modell elmentve marker_classification_model05.pkl fájlba.")

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
