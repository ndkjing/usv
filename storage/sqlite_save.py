#!/usr/bin/python
import sqlite3
import uuid
import threading
import time


class DataBase:
    def __init__(self):
        self.conn = None
        self.current_salary = None

    # 连接数据库
    def connect(self):
        self.conn = sqlite3.connect('test.db', check_same_thread=False)
        print("Opened database successfully")

    # 查找数据
    def select_data(self):
        salary = None
        if self.conn is None:
            self.connect()
        c = self.conn.cursor()
        cursor = c.execute("SELECT id, name, address, salary  from COMPANY")
        for row in cursor:
            # print("ID = ", row[0])
            # print("NAME = ", row[1])
            if row[1] == 'Paul':
                self.current_salary = row[3]
                salary = row[3]
            # print("ADDRESS = ", row[2])
            # print("SALARY = ", row[3], "\n")
        print("Select done successfully")
        self.conn.close()
        self.conn = None
        return salary

    # 插入数据
    def insert_data(self):
        if self.conn is None:
            self.connect()
        c = self.conn.cursor()
        print("Opened database successfully")
        id_ = int(uuid.uuid4())
        value = (id_, 'Paul', 32, 'California', 20000.00)
        c.execute("INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) \
              VALUES " + str(value))
        self.conn.commit()
        print("Records created successfully")
        self.conn.close()
        self.conn = None

    # 更新数据
    def update_data(self):
        self.select_data()
        if self.conn is None:
            self.connect()
        c = self.conn.cursor()
        self.current_salary += 1
        c.execute("UPDATE COMPANY set SALARY = %f where NAME='Paul' " % self.current_salary)
        self.conn.commit()
        print("Total number of rows updated :", self.conn.total_changes)
        # cursor = self.conn.execute("SELECT id, name, address, salary  from COMPANY")
        # for row in cursor:
        #     print("ID = ", row[0])
        #     print("NAME = ", row[1])
        #     print("ADDRESS = ", row[2])
        #     print("SALARY = ", row[3], "\n")
        #
        # print("Operation done successfully")
        self.conn.close()
        self.conn = None

    # 删除数据
    def delete_data(self):
        if self.conn is None:
            self.connect()
        c = self.conn.cursor()
        print("Opened database successfully")

        c.execute("DELETE from COMPANY where ID=2;")
        self.conn.commit()
        print("Total number of rows deleted :", self.conn.total_changes)

        cursor = self.conn.execute("SELECT id, name, address, salary  from COMPANY")
        for row in cursor:
            print("ID = ", row[0])
            print("NAME = ", row[1])
            print("ADDRESS = ", row[2])
            print("SALARY = ", row[3], "\n")

        print("Operation done successfully")
        self.conn.close()
        self.conn = None

    # 创建表
    def create_tabel(self):
        if self.conn is None:
            self.connect()
        c = self.conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS COMPANY
               (ID INT PRIMARY KEY     NOT NULL,
               NAME           TEXT    NOT NULL,
               AGE            INT     NOT NULL,
               ADDRESS        CHAR(50),
               SALARY         REAL);''')
        print("Table created successfully")
        self.conn.commit()
        self.conn.close()
        self.conn = None

    # 测试多线程能否正常执行
    def thread_update_data(self):
        while True:
            self.update_data()
            time.sleep(1)


if __name__ == '__main__':
    sqllite_obj = DataBase()
    # t1 = threading.Thread(target=sqllite_obj.thread_update_data)
    # t2 = threading.Thread(target=sqllite_obj.thread_update_data)
    # t1.start()
    # t2.start()
    # t1.join()
    # t2.join()
    # while True:
    #     time.sleep(1)

    # sqllite_obj.exist_tabel()
    # sqllite_obj.create_tabel()
    # sqllite_obj.insert_data()
    print(sqllite_obj.select_data())
    sqllite_obj.update_data()
    sqllite_obj.select_data()
    sqllite_obj.delete_data()
    sqllite_obj.select_data()
