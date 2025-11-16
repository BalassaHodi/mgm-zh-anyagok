# Gyakorlati anyagok

Egy rövid jegyzet a gyakorlati anyagok tartalmáról, hogy könnyebb legyen megkeresni, hogy mit melyik gyakorlaton vettünk.

Az egyes gyakorlatok alcímei a mappát tartalmazó elemenet jelöli.

Az egyes linkek a nevek mellett elvezetnek az adott mappába/file-ba.

<br>

## GYAK2  - [mappa](/Gyakorlatok/gyak2/)
### Általános info
- Megismerkedtünk, hogy hogyan épül fel egy ros2 node.
- Létrehoztunk két node-ot, az egyik egy publishert a mésik egy subscribert tartalmazó node
- a setup a `setup.py` file-ban

### publisher.py - [file](/Gyakorlatok/gyak2/gyak2/publisher.py)
- tartalmaz egy publishert, ami egy `String` típusú üzenetet küld ki a `/chatter` topicra.
- van benne egy timer is, aminek a callback-jében kiküldünk a terminálra egy üzenetet, illetve publisholjuk az üzenetét a publishernek

<br>

## GYAK3 - [mappa](/Gyakorlatok/gyak3/)
### Általános info
- itt már használunk egy előre felvett rosbag-et, amit rviz-ben lejátszunk
- későbbi gyakorlatokon ezt a rosbag-et használjuk
- a gyakorlaton egy node-ot hoztunk létre, amiben paraméterekkel dolgozunk, illtve gy publisher-t és subscribert is tartalmaz a node.
- a node egy path-t hoz létre az aktuális helyadatokat felhasználva
- itt már launch file-t is hoztunk létre
- illetve rvi file is van
- itt a setup még CMAKELists.txt formában van

### path.py - [file](/Gyakorlatok/gyak3/gyak3/path.py)
- A node-ban először létrehozunk paramétereket, amik tartalmazzák a topic neveket, illetve a egy `max_size` változót, ami a path létrehozásánál használunk
- tartalmaz egy publisher-t, ami egy `Path` üzenetet küld ki a paraméterként bekért topic-ra
- illetve tartalmaz egy subscribert, ami `Odometry` típusú üzenetre iratkozik fel, amiben megkapja a robot aktuális pozícióját
- a subscriber cb-jében amint megkapja a pozíciót, azt a pozíciót kiküldi egy Path-ba

### gyak3.launch.xml - [file](/Gyakorlatok/gyak3/launch/gyak3.launch.xml)
- tartalmazza a rosbaget lejátszásra
- illetve egy node-ot, amit mi hoztunk létre
- illetve az rviz-es node-ot

<br>

## GYAK4 - [mappa](/Gyakorlatok/gyak4/)
### Általános info
- itt igazából nem hoztunk létre node-ot, hanem egy launch file-ban a gyak3-as `path.py` node-ot használtuk és néztük meg rviz-ben és gazebo-ban is
- a setup CMAKEList.txt-ben van
- van egy rviz file is a szimuláláshoz
- van még a config mappában egy yaml file, ebben paraméterek vannak

### gyak4.launch.xml - [file](/Gyakorlatok/gyak4/launch/gyak4.launch.xml)
- argumentumok definiálva a gazeboo szimulációban a robot kezdeti pozíciójához
- a `turtlebot3_gazebo` package `turtlebot3_world` launch file-ja is be van ágyazva. Ez indítja el a gazebo szimulációt
- benne van a gyak3-as node is
- illetve az rviz a szimuláció lejétszásához

<br>

## GYAK5 - [mappa](/Gyakorlatok/gyak5/)
### Általános info
- a package-ben létrehoztunk egy node-ot, amiben vizualizációs dolgokkal ismerkedtünk meg, méghozzá a MarkerArray-el és Markerrel
- launch file-ban dolgoztunk, amiben benne van a gyak3-as rosbag is
- a setup file pythonos formátumban van (setup.py)

### test_viz.py - [file](/Gyakorlatok/gyak5/gyak5/test_viz.py)
- van benne egy paraméter deklarlás, illetve egy subscription és egy publisher
- a subscription egy `Odometry` típusú üzenetre iratkozik fel a `/odom` topicra, amiből a robot aktuális pozícióját kapja meg
- a publisher pedig egy `MarkerArray` típusú üzenetben küld adatokat a `/viz` topicra
- subscriber cb-jében létrehozunk kék téglatesteket a `Marker` üzenet segítségével, amik a robot aktuális pozícióján jelennek meg egy adott időre. Majd ezeket segédfüggvények segítségével kicsit átvariáljuk és a markereket hozzáadjuk egy listához, ami egy `MarkerArray` típusú üzenetben továbbításra kerül a publisher által.
- Az egyik segédfüggvény a `path_publisher` azt csinálja, hogy a bemenő Odometry és MarkerArray üzenetekből létrehoz a robot által bejárt útvonalon gömböket random színekkel.
- a másik seédfüggvény, a `rectangle_publisher` pedig létrehoz egy téglatestet ami befogja az egész pályáját a robotnak, annak maximális x és y pozíciója alapján.

### gyak5.launch.xml - [file](/Gyakorlatok/gyak5/launch/gyak5.launch.xml)
- itt lejátszuk a rosbag-et
- majd a `test_viz.py` node-ot is
- és az rvizet is

<br>

## GYAK6 - [mappa](/Gyakorlatok/gyak6/)
### Általános info
- Itt dolgoztunk a rosbag által generált lidar mérésekkel és azok vizualizációjával
- megismerkedtünk a PointCloud2, LaserScan üzenetekkel, és hogy hogyan kell pontfelhőt létrehozni
- a feladatot launch file-ban csinátuk meg és pythonos setup.py-ban

### test_scan.py - [file](/Gyakorlatok/gyak6/gyak6/test_scan.py)
- itt van 3 pulisher (egy `PointCloud2` típusú, egy `MarkerArray` típusú illetve egy `PoseStamped` típusú)
- illetve egy subscriber ami egy `LaserScan` típusú üzenetben kapja meg az adott pozícióhoz tartozó lidar mérési értékeket
- a suscriber cb-jében miután a `LaserScan` üzenetből az adatokat kiolvastuk (ranges field-ből), az adott mérési pontokra `Marker` segítségével gömböket teszünk és ezeket publisholjuk (`MarkerArray` típusú üzenetben.)
- majd ugyanezt megcsináljuk `PointCloud2`-es üzenetben is. Ezt úgy tesszük meg, hogy a lidar mérésekre létrehozott listából egy pointcloud2 típusú üzenetet csinálunk. Ezután ezt publisholjuk `PointCloud2` üzenetben.
- még van egy olyan része ennek a c függvénynek, hogy kiszámoljuk a legközelebbi pontot, majd ezt a pontot egy `PoseStamped` típusú üzenetben továbbítjuk

### gyak6.launch.xml - [file](/Gyakorlatok/gyak6/launch/gyak6.launch.xml)
- gyak3-as rosbag lindítása
- a létrehozott test_scan node csatolása
- rviz csatolása

<br>

## GYAK7 - [mappa](/Gyakorlatok/gyak7/)
### Általános info
- ebben a package-ben egy differenciűl hajtású robotot és annak vezérlését valósítottuk meg
- ehhez megismerkedtünk a transzformációval, létrehoztunk broadcastert
- megismerkedtünk még a TransformStamped üzenettípussal és a quaternion_from_euler fügvénnyel is
- launch file-t is létrehoztunk, illetve setup.py-t használtunk

### diff_robot.py - [file](/Gyakorlatok/gyak7/gyak7/diff_robot.py)
- a robot differenciál hatását megvalósító node
- feliratkozik egy `Twist` típusú üzenetre, amiből a hosszirányú sebességet kapja meg, illetve publishol egy `Odometry` típusú üzenetre a kiszámolt pozíciót és orientációt
- van még benne egy timer is, ami 20 Hz-en működik
- illetve van egy broadcaster is benne, ami a transzformáióért fontos
- a subscriber cb-jében megkapja a hosszirányú sebességet
- a timer cb-jében pedig kiszámolja a pozíciókat, amit az "odom" frame-ben eltárol egy `Odometry`` típusú üzenetben és kiküldi a publisherbe
- majd pedig létrehoz egy `TransfromStamped` objektumot, amiben pedig kiküldi a transzformált pozíciókat a broadcaster segítségével

### control.py - [file](/Gyakorlatok/gyak7/gyak7/control.py)
- a robot irányítására szolgáló node
- egy pid szabályzást valósít meg, ehhez bekéri a Kp, Ki és Kd értékeket, illetve még maximális sebesség értékeket
- van benne egy `PoseStamped` típusú subscription, amivel a célpozíciót kapja meg
- van benne egy `Odometry` típusú subscription, amiben az aktuális pozíciót kapja meg
- van benne egy publisher, amivel `Twist` típusú üzenetben továbbítja a hosszirányú sebesség értékekt
- illetve van egy timer-je is
- miután a subscription cb-ekben megkapta az aktuális pozíciót és a cél pozíciót, a timer db-jében megy végbe az irányítási logika
- A timer cb-jében kiszámítja a PID controlhoz szükséges jeleket (hibák), majd létrehoz egy longitudinális és egy laterális controlt. A létrejött eredményt (hosszirányú sebességet) kiküldi `Twist` típusú üzenetként

### gyak7.launch.xml - [file](/Gyakorlatok/gyak7/launch/gyak7.launch.xml)
- benne van a diff robot és a control node-ja
- a control node-jában paraméterek definiálva
- benne van még az rviz is

<br>

## GYAK8 - [mappa](/Gyakorlatok/gyak8/)
### Általános info
- ebben a package-ben megismerkedünk jobban a tf-el, hogy hogyan kell transzformálni a framek között
- létrehozunk transzformációt a `tfBuffer` és `tfListener` segítségével, illetve létrehozunk statikus transzformációt is

### test_tf.py - [file](/Gyakorlatok/gyak8/gyak8/test_tf.py)
- beolvasunk egy paramétert ami a target frame nekünk
- majd létrehozzuk a `tfBuffer`-t és `tfListener`-t
- és létrehozunk egy statikus transzformációt a "base_link" és a "safety_left" frame-ek között. Ez azt jelenti, hogy létrehozunk egy olyan koordináta transzformációt, ami a bal kerék mellett 0.5 m-rel hoz létre egy koordináta rendszert
- ezt kiküldjük statikus transzformációként
- van még benne két publisher amik `Odometry` típusú üzenetet küldenek ki. Az egyik az "/odom_left" topic-ra, a másik az "/odom_right" topicra
- van mégegy publisher ami szintén `Odometry` üzenetet küld ki az "/odom_safety_left" topicra
- van egy timer, amiben meghívódik egy belső függvény, ami a pozíciókat transzformálja az adott frame-be és küldi ki
- a belső függvényben (publish_side_position) az történik, hogy megnézi, hogy az adott pozíciót tudja e transzformálni majd transzformálja azt a tfBuffer segítségével és kiküldi az adott publisherben.

### gyak8.launch.xml - [file](/Gyakorlatok/gyak8/launch/gyak8.launch.xml)
- benne van a gyak3-as rosbag
- illetve ez a test_tf node
- illetve a gyak3-ban létrehozott path.py node, amiből 4 is van, csak különböző a nevük és mindig máshova hozzák létre a path-t
- benne van még az rviz is

<br>

## GYAK9 - [mappa](/Gyakorlatok/gyak9/)
### Általános info
- ebben a package-ben egy gazebo szimuláció által generált lidar mérésekekt dolgoztunk fel és pointcloudként vizualizáltuk
- a transzformációt is használtuk a tfBuffer és tfListener segítségével

### scan_tf.py - [file](/Gyakorlatok/gyak9/gyak9/scan_tf.py)
- a node-ban először bekérjük a paramétert a frame elnevezésére
- majd létrehozzuk a tfBuffer-t és tfListenert a transzformációhoz
- illetve van még egy publisher, ami `PointCloud2` üzenetet publishol, illetve két subscriber amik két robotnak a `LaserScan` üzenetét kapják meg
- van még egy timer is
- a subscriberek cb-jében a lézer üzeneteket raktározzuk egy egy point listában, ahol x és y a range-ből jön, z pedig 0
- a timer cb-jében a potokból létrehozunk egy `PointCloud2` üzenetet, majd ezeket transzformáljuk a tfBuffer segítségével
- végül ezt a transzformált pointcloud-ot küldjük ki
- van még egy belső függvény is, ami arra szolgál, hogy összemergeljen két pointcould-ot

### gyak9.launch.xml - [file](/Gyakorlatok/gyak9/launch/gyak9.launch.xml)
- be van ide illesztve a gazebo
- illetve két robot
- illetve két static_transform_publisher node is a robotokhoz
- végül az rviz

### setup.py - [file](/Gyakorlatok/gyak9/setup.py)
- itt a data_files-nál még egy sor be van illesztve a megszokott 3 sor mellé

<br>

## GYAK10 - [mappa](/Gyakorlatok/gyak10/)
### Általános info
- ebben a package-ben egy ackermann kormányzású (első tengely kormányzott) robotnak az implementálása van
- a robotra van írva egy pure pursuit vezérés is
- használunk benne transzformációt, (statikusat és broadcastert is), illetve 

### ack_robot.py - [file](/Gyakorlatok/gyak10/gyak10/ack_robot.py)
- van benne egy subscription `Twist` típusú üzenetre, amiből a hosszirányú sebességet, illetve a legyezési szögsebességet kapja meg
- van egy publisher `Odometry` típusú üzenetre, amiben a pozíciót küldi ki
- van benne még egy dinamikus broadcaster, illetve egy statikus broadcaster is
- a statikus broadcaster a hátsó tengelynek a pozícióját hozza létre az első tengelyhez képest fix x eltolással
- van egy timer is, aminek a cb-jében kiszámíítjuk a pozíciókat és az elfordulást, és ezeket publisholjuk. Illetve létrehozzuk a broadcaster-t, ami az odomból a base_link frame-be hozza létre dinamikusan az x-y pozíciót és az orientációt

### pure_pursuit.py - [file](/Gyakorlatok/gyak10/gyak10/pure_pursuit.py)
- először bekérjük a paramétereket, majd létrehozunk egy tfBuffert és tfListenert a transzformációhoz
- van egy publisher, ami `Twist` típusú üzenetet továbbít. Ebben van benne a létrehozott hosszirányú sebesség és legyezési szögsebesség
- van mégegy publisher ami pedig `MarkerArray` üzenetben továbbít vizualizációs adatokat
- van egy subscriber, ami `Path` üzenetben megkapja a követendő útvonalat
- van egy timer is benne, aminek a cb-jében először is létrehozzuk az aktuális pozíciót a tfBuffer segítségével
- majd a pure pursuit vezérléshez megkeressük a path-ban a legközelebbi pontot a hátsó tengelyhez, amit a belső send_point függvénnyel ki is küldünk
- illetve megkeressük a lookahead pontot is, ami a pure pursuit irányításhoz kell, és ezt is kiküldjük azzal a függvénnyel
- majd a pure pursuit vezérlést megtervezzük
- végül kiküldjük a létrejött vezérlési inputot a `Twist` típusú üzenetben
- a send_point belső függvény egy `MarkerArray`-ban küldi ki a megjelenítendő pontokat (egy piros gömbként)

### gyak10.launch.xml - [file](/Gyakorlatok/gyak10/launch/gyak10.launch.xml)
- benne a tf node
- illetve a pálya publisholó node-ja egy másik packageből
- illetve a robot és a pure pursuit node-ja
- és a gyak3-as pálya is
- végén pedig az rviz

<br>

## GYAK11 - [mappa](/Gyakorlatok/gyak11/)
### Általános info
- Ez egy ZH gyakorlás volt, aminek a Doba D. féle megoldása itt, az általam készített megoldása a ZH gyakorlások mappában van

<br>

## GYAK12 - [mappa](/Gyakorlatok/gyak12/)
### Általános info
- ez is egy mintaZH, aminek az általam készíett megoldása a ZH gyakorlások mappában van