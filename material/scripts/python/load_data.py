#!/usr/bin/env python3








from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import pandas as pd
import numpy as np








def load_file(filename: str) -> pd.DataFrame:
 '''
 Load a csv file into a pandas dataframe and strip spaces from column names








 Parameters
 ----------
 - `filename`: path to the csv file








 Returns
 -------
 - `df`: pandas dataframe
 '''








 # Load the data
 df = pd.read_csv(filename, sep=',')








 # strip spaces from column names
 df.rename(columns=lambda x: x.strip(), inplace=True)








 # drop the last column (empty)
 df = df.drop(df.columns[-1], axis=1)








 return df








def plot_pose_groundtruth(df:pd.DataFrame):
 '''
 Plot the pose ground truth
 '''


 fig = plt.figure(figsize=(6,5))
 shape = (2,2)




 ax = plt.subplot2grid(shape, (0,0))
 ax.set_title('Position')
 ax.plot(df.x, df.y)
 l_min = min(*ax.get_xbound(), *ax.get_ybound())
 l_max = max(*ax.get_xbound(), *ax.get_ybound())
 ax.set_xlim(l_min,l_max)
 ax.set_ylim(l_min,l_max)
 ax.set_xlabel('x [m]')
 ax.set_ylabel('y [m]')
 ax.grid()


 ax = plt.subplot2grid(shape, (0,1))
 ax.set_title('Heading')
 ax.plot(df.time, df.heading)
 ax.set_xlabel('time [s]')
 ax.set_ylabel('[rad]')
 ax.grid()


 ax = plt.subplot2grid(shape, (1,0))
 ax.set_title('Velocity')
 ax.plot(df.time, df.vel_x, label='$\dot{x}$')
 ax.plot(df.time, df.vel_y, label='$\dot{y}$')
 ax.set_xlabel('time [s]')
 ax.set_ylabel('[m/s]')
 ax.legend()
 ax.grid()




 ax = plt.subplot2grid(shape, (1,1))
 ax.set_title('Angular velocity')
 ax.plot(df.time, df.vel_heading)
 ax.set_xlabel('time [s]')
 ax.set_ylabel('[rad/s]')
 ax.grid()




 plt.tight_layout()
 plt.savefig("wp3_pose_ground_truth.png", dpi=300)




 plt.plot(df.x, df.y)
 l_min = min(*ax.get_xbound(), *ax.get_ybound())
 l_max = max(*ax.get_xbound(), *ax.get_ybound())
 ax.set_xlim(l_min,l_max)
 ax.set_ylim(l_min,l_max)
 ax.set_xlabel('x [m]')
 ax.set_ylabel('y [m]')
 ax.grid()










def plot_lights_classification(gt: pd.DataFrame, lights: pd.DataFrame):
 status_names = {
     0: "Good",
     1: "Flicker",
     2: "Blinking"
 }


 status_colors = {
     0: "green",
     1: "orange",
     2: "red"
 }


 plt.figure(figsize=(8, 5))


 plt.plot(
     gt["x"],
     gt["y"],
     linewidth=2,
     label="Ground truth trajectory"
 )




 for status in sorted(lights["status"].dropna().unique()):
     subset = lights[lights["status"] == status]
     status_int = int(status)


     plt.scatter(
         subset["x_est"],
         subset["y_est"],
         s=90,
         color=status_colors.get(status_int, "black"),
         label=status_names.get(status_int, "Unknown"),
         edgecolors="black"
     )


     for i, row in subset.iterrows():
         plt.text(
             row["x_est"] + 0.05,
             row["y_est"] + 0.05,
             f"{i + 1}",
             fontsize=8
         )




 plt.xlabel("x position [m]")
 plt.ylabel("y position [m]")
 plt.title("WP3 - Detected lights and defect classification")
 plt.legend()
 plt.grid(True)
 plt.axis("equal")
 plt.tight_layout()
 plt.savefig("wp3_lights_classification.png", dpi=300)










def plot_light_signal(df: pd.DataFrame):
 df["time"] = pd.to_numeric(df["time"], errors="coerce")
 df["light_value"] = pd.to_numeric(df["light_value"], errors="coerce")
 df = df.dropna(subset=["time", "light_value"])


 plt.figure(figsize=(10, 4))
 plt.plot(df["time"], df["light_value"], label="Light signal")
 plt.ylim(bottom=1.3)
 plt.xlabel("Time [s]")
 plt.ylabel("Light intensity")
 plt.title("Raw light signal")
 plt.grid(True)
 plt.legend()
 plt.tight_layout()
 plt.savefig("wp3_raw_light_signal.png", dpi=300)








def plot_fft(df: pd.DataFrame):
 df["id_light"] = pd.to_numeric(df["id_light"], errors="coerce")
 df["freq"] = pd.to_numeric(df["freq"], errors="coerce")
 df["fft_mag"] = pd.to_numeric(df["fft_mag"], errors="coerce")
 df = df.dropna(subset=["id_light", "freq", "fft_mag"])




 lights_to_plot = [1, 2, 6]
 names = {
     1: "Good",
     2: "Flickering",
     6: "Blinking"
 }


 fig, ax = plt.subplots(figsize=(10, 5))


 for light_id in lights_to_plot:
     dfi = df[df["id_light"] == light_id].copy().sort_values("freq")




     if dfi.empty:
         print(f"Warning: no data for light {light_id}")
         continue




     ax.plot(
         dfi["freq"],
         dfi["fft_mag"],
         marker="o",
         label=f"Light {light_id} - {names.get(light_id, 'Unknown')}"
     )


   # Blinking window
 ax.axvspan(
       1.3,
       1.7,
       color="green",
       alpha=0.15,
       label="Blinking window"
   )


   # Flickering window
 ax.axvspan(
       10.8,
       11.4,
       color="orange",
       alpha=0.15,
       label="Flickering window"
   )




 ax.set_xlabel("Frequency [Hz]")
 ax.set_ylabel("FFT magnitude")
 ax.set_title("FFT of Three Light Types")
 ax.grid(True)
 ax.legend()
 plt.tight_layout()
 plt.savefig("wp3_fft_of_three_light_types.png", dpi=300)








# ==============================
# temperature plot
#===============================




def plot_temperature(df: pd.DataFrame, q_heater=0.1, C=0.25, T_SET=21.5, final_fraction=0.2):
  """
  Traite les données brutes de température, identifie les paramètres du modèle
  thermique de la serre, et génère le graphique comparatif.
  """
  # 1. Agrégation spatio-temporelle (Moyenne des capteurs par pas de temps)
  df_time = df.groupby("time", as_index=False).agg(
      {"tin": "mean", "tout": "mean", "signal_strength": "mean"}
  )
   t = df_time["time"].values
  tin = df_time["tin"].values
  tout = df_time["tout"].values
   # 2. Identification de la résistance thermique R à l'état stationnaire
  n = len(df_time)
  n_final = max(5, int(final_fraction * n))
  final_data = df_time.iloc[-n_final:]
   tin_final = final_data["tin"].mean()
  tout_final = final_data["tout"].mean()
   R_est = (tin_final - tout_final) / q_heater
   # 3. Calcul de la résistance thermique idéale R*
  R_ideal = (T_SET - tout_final) / q_heater
   # 4. Modélisation de la réponse indicielle (système du 1er ordre)
  t0 = t[0]
  Tin0 = tin[0]
  tin_model = tout_final + R_est * q_heater + (Tin0 - tout_final - R_est * q_heater) * np.exp(-(t - t0) / (R_est * C))
   # 5. Restitution des résultats numériques
  print("=== Synthèse de l'Identification Thermique ===")
  print(f"Échantillons temporels exploités : {len(df_time)} (sur {len(df)} paquets bruts)")
  print(f"Tin_inf (régime permanent)       = {tin_final:.3f} °C")
  print(f"Tout (moyenne extérieure)        = {tout_final:.3f} °C")
  print(f"Résistance thermique estimée (R) = {R_est:.3f} °C/W")
  print(f"Résistance thermique cible (R*)  = {R_ideal:.3f} °C/W")
  print("Diagnostic : Isolation " + ("suffisante" if R_est >= R_ideal else "défaillante"))




  # 6. Génération et sauvegarde du graphique
  plt.figure(figsize=(10, 6))
  plt.plot(t, tin, label="Température intérieure mesurée $T_{in}$", linewidth=2)
  plt.plot(t, tout, label="Température extérieure $T_{out}$", linewidth=2)
  plt.plot(t, tin_model, "--", label="Modèle analytique $\hat{T}_{in}(t)$", linewidth=2)
   plt.xlabel("Temps [s]")
  plt.ylabel("Température [°C]")
  plt.title(f"Caractérisation Thermique de la Serre\n$R_{{est}} = {R_est:.3f}$ °C/W | $R^* = {R_ideal:.3f}$ °C/W")
  plt.grid(True, linestyle=":", alpha=0.7)
  plt.legend()
  plt.tight_layout()
  plt.savefig("thermal_response.png", dpi=300)




if __name__ == "__main__":




 TEMP_PATH = "../../controllers/controller/data/temperature_log.csv"
 GT_PATH = "../../controllers/supervisor/data/ground_truth.csv"
 SIGNAL_PATH = "../../controllers/controller/data/light_signal.csv"
 FFT_PATH = "../../controllers/controller/data/light_fft.csv"
 LIGHT_PATH = "../../controllers/controller/data/light_detect.csv"




 temp = load_file(TEMP_PATH)
 gt = load_file(GT_PATH)
 signal = load_file(SIGNAL_PATH)
 fft = load_file(FFT_PATH)
 lights = load_file(LIGHT_PATH)




 print("Temperature columns:", temp.columns.tolist())
 print("Ground truth columns:", gt.columns.tolist())
 print("Signal columns:", signal.columns.tolist())
 print("FFT columns:", fft.columns.tolist())
 print("Light columns:", lights.columns.tolist())




 plot_temperature(temp, q_heater=0.1, C=0.25, T_SET=21.5, final_fraction=0.2)
 plot_pose_groundtruth(gt)
 plot_light_signal(signal)
 plot_fft(fft)
 plot_lights_classification(gt, lights)








 plt.show()

















