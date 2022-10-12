1. To copy file into robot root

```bash
scp test_robot.py root@172.24.1.1:/
scp test_lidar.py root@172.24.1.1:/
scp astar.py root@172.24.1.1:/
scp lidar.py root@172.24.1.1:/
scp robot.py root@172.24.1.1:/
scp -r AlvesArthautDang root@172.24.1.1:/
```

rm -r AlvesArthautDang

2. Open ssh root and move file

```bash
ssh root@172.24.1.1
mv astar.py .
mv lidar.py .
mv /AlvesArthautDang .
```

3. Move file into robot root

```bash
mv /test_robot.py .
```

4. Run project

```bash
python3 astar_project/robot.py
```
