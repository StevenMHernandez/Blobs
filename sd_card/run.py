import os

if __name__ == "__main__":
    print("sup")
    f = open("all.txt")
    for line in f.readlines():
        print(line)
        spl = line.split(":")
        uid = spl[1].strip().split(" ")
        msg = spl[2].strip()
        print(uid)
        print(msg)

        dir_prefix = "./data"
        for hex in uid:
            dir_prefix += "/" + hex
            print("ensuring directory exists:", dir_prefix)
            if not os.path.exists(dir_prefix):
                os.makedirs(dir_prefix)

        f_sub = open(dir_prefix + "/msg.txt", 'w')
        f_sub.write(msg)
        f_sub.close()