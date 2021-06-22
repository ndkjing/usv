import unittest
from unittest import TestCase
from externalConnect import baidu_map


class Test(TestCase):
    def __init__(self,*args, **kwargs):
        super(Test, self).__init__(*args, **kwargs)
        self.src_point = [114.4314, 30.523558]

    def test_get_pool_name(self):
        src_point = [114.4314, 30.523558]
        self.baidu_obj = baidu_map.BaiduMap(src_point, zoom=15,
                                            scale=1, map_type=baidu_map.MapType.gaode)
        self.baidu_obj.get_pool_name()
        self.assertEqual(type(self.baidu_obj.pool_name), str)
        self.assertEqual(type(self.baidu_obj.address), str)

    def test_get_pool_pix(self):
        src_point = [114.4314, 30.523558]
        self.baidu_obj = baidu_map.BaiduMap(src_point, zoom=15,
                                            scale=1, map_type=baidu_map.MapType.gaode)
        pool_cnts, (pool_cx, pool_cy) = self.baidu_obj.get_pool_pix(b_show=False)
        self.assertIsNot(pool_cnts, None)

    def test_scan_pool(self):
        src_point = [114.4314, 30.523558]
        self.baidu_obj = baidu_map.BaiduMap(src_point, zoom=15,
                                            scale=1, map_type=baidu_map.MapType.gaode)
        pool_cnts, (pool_cx, pool_cy) = self.baidu_obj.get_pool_pix(b_show=False)
        scan_cnts = self.baidu_obj.scan_pool(meter_gap=50, safe_meter_distance=10, b_show=False)
        self.assertFalse(len(scan_cnts) < 0)

    # def test_get_pool_name(self):
    #     self.baidu_obj = baidu_map.BaiduMap(self.src_point, zoom=15,
    #                                         scale=1, map_type=baidu_map.MapType.gaode)
    #     print(self.baidu_obj.get_pool_name())


if __name__ == '__main__':
    unittest.main()



