from unittest import TestCase
from utils import data_valid
import config
import unittest


class Test(TestCase):
    def test_get_current_water_data(self):
        data_valid.get_current_water_data()

        return_list = data_valid.get_water_data(config.WaterType.wt)
        self.assertEqual(type(return_list),list)
        return_list = data_valid.get_water_data(config.WaterType.EC)
        self.assertEqual(type(return_list),list)
        return_list = data_valid.get_water_data(config.WaterType.pH)
        self.assertEqual(type(return_list), list)
        return_list = data_valid.get_water_data(config.WaterType.DO)
        self.assertEqual(type(return_list), list)
        return_list = data_valid.get_water_data(config.WaterType.TD)
        self.assertEqual(type(return_list), list)
        return_list = data_valid.get_water_data(config.WaterType.NH3_NH4)
        self.assertEqual(type(return_list), list)




if __name__ == '__main__':
    unittest.main()