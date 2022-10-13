## Connect to Minimox

1. To copy file into robot root

```bash
scp -r AlvesArthautDang root@172.24.1.1:/
```

2. Open ssh root and move file

```bash
ssh root@172.24.1.1
mv /AlvesArthautDang .
rm -r AlvesArthautDang
```

3. Move file into robot root

```bash
mv /AlvesArthautDang.py .
```

4. Run project

```bash
python3 AlvesArthautDang/robot.py
```

## Step by step of coding

1. Write function Astar
2. Try to get data from CSV (angle, dist) => convert to catesian (x, y)
   => matrice[10][10] => put Astar into the matrice
3. Scan Lidar data (angle, dist) => (x, y)
   => matrice[10][10] => put Astar in matrice
   - Point start = (0, 5)
   - Point end (9, 5)
   - Matrice dimension 10x10
4. Take 2 first element from Astar => convert to position
   - Exemple: path_astar = [(0, 5), (0, 6)] => path_robot = [(0 - 0, (6 - 5))] = [(0, 1)] => LEFT
5. Send to robot the position to go
   - (0, 1) => LEFT => "q"
   - (0, -1) => RIGHT => "d"
   - (1, 0) => UP => "z"
   - (-1, 0) => DOWN => "s"
6. Create a boucle infinity for step 3 => 5
