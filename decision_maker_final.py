import math

# Carla (oder Commonroad) -Path einlesen
path = [(106.1204605102539, 27.85234260559082), (105.7789535522461, 25.622909545898438),
        (105.11431121826172, 23.467626571655273), (104.14088439941406, 21.433061599731445),
        (102.87971496582031, 19.563180923461914), (101.35881805419922, 17.899057388305664),
        (99.63081359863281, 16.476526260375977), (97.72457885742188, 15.30355453491211),
        (95.67589569091797, 14.402172088623047), (93.5232162475586, 13.789297103881836),
        (91.30697631835938, 13.476435661315918), (89.18740844726562, 13.431788444519043),
        (87.18741607666016, 13.426231384277344), (85.18742370605469, 13.420674324035645),
        (83.18729400634766, 13.415116310119629), (81.18730163574219, 13.40955924987793),
        (79.18730926513672, 13.404003143310547), (77.18731689453125, 13.398445129394531),
        (75.18732452392578, 13.392889022827148), (73.18733215332031, 13.387331008911133),
        (71.18733978271484, 13.38177490234375), (69.18734741210938, 13.376216888427734),
        (67.1873550415039, 13.370660781860352), (65.18736267089844, 13.365102767944336),
        (63.187374114990234, 13.359546661376953), (61.187381744384766, 13.353988647460938),
        (59.1873893737793, 13.348432540893555), (57.18739318847656, 13.342874526977539),
        (55.187400817871094, 13.337320327758789), (53.187408447265625, 13.331762313842773),
        (51.187416076660156, 13.32620620727539), (49.18742752075195, 13.320646286010742),
        (47.187435150146484, 13.31509017944336), (45.187442779541016, 13.309532165527344),
        (43.18745040893555, 13.303976058959961), (41.18745803833008, 13.298418045043945),
        (39.18746566772461, 13.292861938476562), (37.18747329711914, 13.287303924560547),
        (35.18748092651367, 13.281747817993164), (33.18748474121094, 13.276189804077148),
        (31.18749237060547, 13.270633697509766), (29.1875, 13.26507568359375), (27.187509536743164, 13.259519577026367),
        (25.187517166137695, 13.253963470458984), (23.187524795532227, 13.248405456542969),
        (21.187536239624023, 13.242849349975586), (19.187543869018555, 13.23729133605957),
        (17.187551498413086, 13.231735229492188), (15.187560081481934, 13.226177215576172),
        (13.187567710876465, 13.220621109008789), (11.187575340270996, 13.215063095092773),
        (9.187582969665527, 13.20950698852539), (7.1875901222229, 13.203948974609375),
        (5.187597751617432, 13.198392868041992), (3.18760347366333, 13.192834854125977),
        (1.1876111030578613, 13.187278747558594), (-0.8123793005943298, 13.181720733642578),
        (-2.8123717308044434, 13.176164627075195), (-4.812364101409912, 13.17060661315918),
        (-6.812356472015381, 13.165050506591797), (-8.812348365783691, 13.159494400024414),
        (-10.81234073638916, 13.153936386108398), (-12.812333106994629, 13.148380279541016),
        (-14.812325477600098, 13.142822265625), (-16.812318801879883, 13.137266159057617),
        (-18.81231117248535, 13.131708145141602), (-20.81230354309082, 13.126152038574219),
        (-22.81229591369629, 13.120594024658203), (-24.812288284301758, 13.11503791809082),
        (-26.812280654907227, 13.109479904174805), (-28.812273025512695, 13.103922843933105),
        (-30.672256469726562, 13.061885833740234), (-32.38929748535156, 12.790976524353027),
        (-34.04729080200195, 12.268775939941406), (-35.60966873168945, 11.506799697875977),
        (-37.04197311401367, 10.521851539611816), (-38.313087463378906, 9.335165977478027),
        (-39.42044448852539, 7.962237358093262), (-40.33416748046875, 6.453503608703613),
        (-41.0378303527832, 4.836092472076416), (-41.51879119873047, 3.139080286026001),
        (-41.768394470214844, 1.3929791450500488), (-41.807823181152344, -0.48009511828422546),
        (-41.813446044921875, -2.480087995529175), (-41.81907653808594, -4.4800801277160645),
        (-41.824703216552734, -6.480072021484375), (-41.83195495605469, -8.493301391601562),
        (-41.84316635131836, -10.494643211364746), (-41.85200500488281, -12.478289604187012),
        (-41.85272979736328, -14.456018447875977), (-41.84506607055664, -16.433731079101562),
        (-41.83060073852539, -18.42555046081543), (-41.81550979614258, -20.425493240356445),
        (-41.800418853759766, -22.42543601989746), (-41.78532791137695, -24.425378799438477),
        (-41.77023696899414, -26.425323486328125), (-41.75514602661133, -28.425264358520508),
        (-41.740055084228516, -30.425209045410156), (-41.7249641418457, -32.42515563964844),
        (-41.70987319946289, -34.42509460449219)]

# festgelegte Parameter
frequency_step = 1000 #Frequenz in Hertz
t_step = 1 / frequency_step  # Schrittzeit in Abhängigkeit der Frequenz
v_max = 50 / 3.6  # 50km/h maximale Geschwindigkeit auf der Geraden
v_curve = 30 / 3.6  # 30km/h maximale Kurvengeschwindigkeit
a_max = 0.4 * 9.81  # 0,4g maximale Beschleunigung
a_brake = -0.4 * 9.81 # -0,4g maximale Brems-Beschleunigung
start_point = path[0]   #Start-Punkt, um x- und y-Koordinate des Startes auszulesen

# Variablen zur aktuellen Position
v_act = 0   #aktuelle Geschwindigkeit
x_act = start_point[0]  #aktuelle x-Position
y_act = start_point[1]  #aktuelle y-Position

path_length = len(path)  # Anzahl der Wegpunkte des ursprünglichen Pfades
new_path = []   #Neuer Path wird als Liste angelegt
new_path.append(start_point)    #Startpunkt wird zum neuen Path hinzugefügt
v_path = [] #Liste mit den Geschwindigkeiten des Fahrzeugs für jeden Wegpunkt
v_path.append(v_act)    #Die Anfangsgeschwindigkeit wird zum Geschwindigkeits-Pfad hinzugefügt

# Winkel der Wegpunkte
angle = []  #Liste mit den Winkeln zwischen einem Punkt und den beiden folgenden Wegpunkten
for i in range(path_length - 2):
    point = path[i] #aktueller Wegpunkt
    point2 = path[i + 1]    #nächster Wegpunkt
    point3 = path[i + 2]    #übernächster Wegpunkt
    alpha = math.atan((point3[0] - point[0]) / (point3[1] - point[1]))  # Winkel zwischen dem aktuellen und dem übernächsten Punkt (Für Winkelmaß: *180/math.pi)
    beta = math.atan((point2[0] - point[0]) / (point2[1] - point[1]))  # Winkel zwischen dem aktuellen und dem nächsten Punkt (Für Winkelmaß: *180/math.pi)
    gamma = alpha - beta    #Differenz der Winkel, um den Winkel des Dreiecks zu berechnen
    angle.append(gamma) #Winkel wird zur Liste hinzugefügt


# Maximale Geschwindigkeit am jeweiligen Wegpunkt
v_max_point=[]  #Liste mit den maximalen Geschwindigkeiten eines jeden Wegpunkts des ursprünglichen Pfades
for i in range(path_length-2):      #Für jeden zuvor berechneten Winkel wird eine maximale Geschwindigkeit berechnet
    v_max_point.append(v_max - ((v_max - v_curve) * abs(angle[i] / max(angle))))    #Abhängig des Winkels, wird die Geschwindigkeit verringert

stop_flag=0     #Punkt an dem der restliche Weg nicht mehr ausreicht, um abzubremsen
for i in range(path_length-3):
    way_brake = (a_brake / 2) * ((-v_max_point[i] / a_brake) ** 2) + v_max_point[i] * (-v_max_point[i] / a_brake)   #Benötigter Weg, um mit der aktuellen Geschwindigkeit auf 0 abzubremsen
    brake_distance=0    #Übriger Weg, bis zum Zielpunkt
    for j in range(path_length-1-i): #Die Distanz von jedem Punkt bis zum jeweils nächsten wird errechnet
        point1 = path[j+i]
        point2 = path[j+i+1]
        brake_distance = brake_distance + math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)   #Übriger Weg wird aufaddiert
    if way_brake>brake_distance:    #Abfrage, ob der benötigte Bremsweg vorhanden ist
        stop_flag=i    #Der Punkt ab dem abgebremst werden muss (folgend: Bremspunkt), wird gespeichert
        break   #Um performance einzusparen, wird die Schleife beendet
for i in range(path_length-stop_flag-1):   #Die maximalen Geschwindigkeiten der Wegpunkte ab dem Bremspunkt werden gelöscht
    del v_max_point[-1]
v_diff_brake=v_max_point[-1]    #Geschwindigkeit im Bremspunkt ab der abgebremst wird
for i in range(path_length-stop_flag+1):    #Für die fehlenden Wegpunkte wird die maximale Geschwindigkeit berechnet
    v_max_point.append(v_diff_brake - i/(path_length-stop_flag)*v_diff_brake)   #Die maximale Geschwindigkeit wird gleichmäßig verringert

# path-Berechnung
for i in range(len(path) - 1):  #Jeder Wegpunkt des ursprünglichen Paths wird durchgegangen
    #Zur Vereinfachung wird der Weg zwischen zwei Wegpunkten als Gerade angesehen
    point = new_path[-1]    #Der letzte Wegpunkt des neuen Paths wird als Ausgangspunkt der Gerade genommen
        #Es muss der letzte Punkt des neuen Paths genommen werden, da  in vielen Fällen die Gerade nicht die exakte Länge hat, wie der Abstand der Wegpunkte
    point2 = path[i + 1]    #Der darauf folgende Punkt wird als Endpunkt der Gerade genommen
    dis_to_next_point = math.sqrt((point2[0] - point[0]) ** 2 + (point2[1] - point[1]) ** 2) #Die Länge der Gerade wird berechnet
    time_to_next_point = dis_to_next_point / ((v_act + v_max_point[i + 1]) / 2) #Anhand der Durchschnittsgeschwindigkeit wird die benötigte Zeit berechnet
    a_act = (v_max_point[i+1] - v_act) / time_to_next_point #Mit Geschwindigkeitsdifferenz und der Zeit wird die Beschleunigung berechnet

    if a_act > a_max:   #Falls die Beschleunigung zu groß wird, wird sie hier begrenzt
        a_act = a_max
        time_root = math.sqrt((v_act / a_act) ** 2 + (2 * dis_to_next_point / a_act))   #Durch einen andere Beschleunigung, wird eine neue Zeit für den Weg berechnet
        if time_root > (-v_act / a_act):    #p-q-Formel für Zeitberechnung muss einen physikalisch-sinnvollen Wert (größer 0) ergeben
            time_to_next_point = -v_act / a_act + time_root
        else:
            time_to_next_point = -v_act / a_act - time_root
    elif a_act < a_brake:   #Falls eine negative Beschleunigung zu groß wird, wird sie hier auf die Bremsbeschleunigung begrenzt
        a_act = a_brake
        time_root = math.sqrt((v_act / a_act) ** 2 + (2 * dis_to_next_point / a_act))   #Neue Zeit wird berechnet (siehe oben)
        if time_root > (-v_act / a_act):
            time_to_next_point = -v_act / a_act + time_root
        else:
            time_to_next_point = -v_act / a_act - time_root

    deltax = point2[0] - point[0]   #Differenz der x-Koordinaten (später wichtig für Vektoren-Rechnung)
    deltay = point2[1] - point[1]   #Differenz der y-Koordinaten (später wichtig für Vekotren-Rechnung)
    num_points = math.ceil(time_to_next_point / t_step) #Anzahl der neuen Wegpunkte, die zwischen den ursprünglichen Punkten benötigt werden
    deltav= (v_act+a_act*time_to_next_point)-v_act  #Geschwindigkeitsänderung zwischen Start- und Endpunkt

    for j in range(num_points): #In jedem Durchlauf wird ein neuer Punkt des neuen Pfades hinzugefügt
        if deltav == 0: #Falls es keine Geschwindigkeitsänderung gibt, wird eine konstante Bewegung angenommen
            way = v_act * t_step    #s=v*t
        else:   #Falls es eine Geschwindigkeitsänderung gibt, wird eine konstant-beschleunigte-Bewegung angenommen
            way = a_act / 2 * (t_step ** 2) + v_act * t_step    #s=a/2*t^2+v0*t
            v_act = v_act + deltav/num_points   #v=v0+vDiff
        x_act = x_act + (way / dis_to_next_point) * deltax  #Auf die x-Koordinate wird ein entsprechend langer Vektor (deltax) aufaddiert
        y_act = y_act + (way / dis_to_next_point) * deltay  #Auf die y-Koordinate wird ein entsprechend langer Vektor (deltay) aufaddiert
        new_point = (x_act, y_act)  #Ein Tuple mit dem neuen Punkt wird erstellt (wichtig für Formatierung in Carla)
        new_path.append(new_point)  #Der Tuple mit dem neuen Wegpunkt wird zur Liste hinzugefügt
        v_path.append(v_act)    #Die aktuelle Geschwindigkeit wird zur Geschwindigkeits-Liste hinzugefügt

v_path[-1]=0    #Damit das Auto in jedem Fall steht, wird die Geschwindigkeit im Ziel auf 0 km/h festgelegt