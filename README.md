# vs_disparity_extender

ROS 2 paket s implementacijom Disparity Extender algoritma za autonomno upravljanje modelom vozila Roboracer.

## Opis implementacije

Algoritam obrađuje 2D LiDAR mjerenja u realnom vremenu:
1. **Predobrada očitanja**: Izdvaja se prednje vidno polje od 180° ispred vozila (`fov_deg`). Neispravne i beskonačne vrijednosti (`inf`, `nan`) zamjenjuju se maksimalnim dosegom (`max_range`).
2. **Detekcija dispariteta i proširenje**: Pronalaze se skokovi u udaljenostima susjednih zraka koji premašuju zadani prag (`disparity_threshold`). Bliža prepreka se proširuje preko daljih zraka za širinu automobila i sigurnosnu marginu (`clearance`), čime se sprječava zapinjanje ruba vozila.
3. **Sigurnosni balon**: Oko točke s najmanjom izmjerenom udaljenosti definira se radijus unutar kojeg se zrake postavljaju na nulu (`bubble_radius`).
4. **Odabir smjera kretanja**: Među preostalim slobodnim prolazima odabire se najširi raspon zraka koje imaju najveću dubinu (`far_ratio`). Ciljani kut upravljanja postavlja se u središte tog raspona.
5. **Upravljanje**: Kut upravljanja se ograničava na maksimalni mehanički zakret kotača (`max_steer_rad`), a brzina se dinamički smanjuje u oštrijim zavojima.

## Popis poboljšanja

Implementacija prati osnovnu ideju algoritma s predavanja uz dinamičko prilagođavanje brzine ovisno o kutu zakreta volana radi stabilnijeg prolaska kroz zavoje te RViz markere za lakše praćenje ciljanog smjera. Svi parametri su u potpunosti konfigurabilni preko YAML datoteke prema zahtjevima zadaće.

## Upute za build i pokretanje

Paket se pokreće unutar Docker kontejnera simulatora (`roboracer-sim`).

### 1. Build paketa:
U terminalu uđite u kontejner simulatora i izgradite paket:
```bash
docker exec -it roboracer-sim bash
cd /sim_ws
colcon build --symlink-install --packages-select vs_disparity_extender
source install/setup.bash
```

### 2. Pokretanje:
1. **U prvom terminalu** uđite u kontejner i pokrenite simulator staze:
   ```bash
   docker exec -it roboracer-sim bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py map_path:=maps/levine_blocked
   ```

2. **U drugom terminalu** uđite u kontejner i pokrenite čvor:
   ```bash
   docker exec -it roboracer-sim bash
   source /sim_ws/install/setup.bash
   ros2 launch vs_disparity_extender disparity_extender_launch.py
   ```

3. Vizualizacija je dostupna u web pregledniku na adresi: `http://localhost:8080/vnc.html`
