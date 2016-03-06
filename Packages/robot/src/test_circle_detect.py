import unittest
from pprint import pprint
from CircleDetect import *

class TestObjectDetection(unittest.TestCase):
    def setUp(self):
        with open("test_input/data_wo_circle.json") as outfile:
            self.laser_data_wo_circle = json.load(outfile)
        with open("test_input/data_w_circle.json") as outfile:
            self.laser_data_w_circle = json.load(outfile)

    def test_segregate_lines(self):
        resp = segregate_lines(self.laser_data_wo_circle)
        line_counter, circle_counter = 0, 0
        for item in resp:
            if item["type"] == "circle":
                circle_counter += 1
            else:
                line_counter += 1
        self.assertEqual(circle_counter, 0)
        self.assertGreater(line_counter, 3)
        
    def test_find_circle(self):
        resp = find_circle(self.laser_data_w_circle)
        circle_counter, line_counter = 0, 0
        for item in resp:
            if item["type"] == "circle":
                circle_counter += 1
            else:
                line_counter += 1
        import pdb; pdb.set_trace()
        self.assertEqual(circle_counter, 1)

    def test_segregate_wo_cirle(self):
        resp = segregate(self.laser_data_wo_circle)
        circle_counter, line_counter = 0, 0
        for item in resp:
            if item["type"] == "circle":
                circle_counter += 1
            else:
                line_counter += 1
        self.assertEqual(circle_counter, 0)
 
    def test_segregate_w_cirle(self):
        resp = segregate(self.laser_data_w_circle)
        circle_counter, line_counter = 0, 0
        for item in resp:
            if item["type"] == "circle":
                circle_counter += 1
            else:
                line_counter += 1
        self.assertEqual(circle_counter, 1)
        


if __name__ == "__main__":
    unittest.main()
