import os


def clear(filepath):
    files = os.listdir(filepath)
    for fd in files:
        cur_path = os.path.join(filepath, fd)
        if os.path.isdir(cur_path):
            # if fd == "__pycache__":
            #     print("rm -rf {}".format(cur_path))
            #     os.system("rm -rf {}".format(cur_path))
            #     os.removedirs(cur_path)
            # else:
            #     clear(cur_path)
            clear(cur_path)
        elif os.path.isfile(cur_path):
            if ".pyc" in fd:
                print("rm -rf {}".format(cur_path))
                # os.remove(cur_path)
                os.remove(cur_path)
            # elif ".gitignore" in fd:
            #     print("rm -rf {}".format(cur_path))
            #     os.remove(cur_path)


if __name__ == "__main__":
    clear(r"F:\pythonProject\usv")
