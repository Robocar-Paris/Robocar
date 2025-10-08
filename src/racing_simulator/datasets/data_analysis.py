import pandas as pd



df = pd.read_csv("data_clean_complete_4_loop.csv", header=0, index_col=0)

print("Aperçu des données :")
print(df.head())

print("\nStatistiques descriptives :")
print(df.describe())

print("\nInformations sur les données :")
print(df.info())

# Vérifier le nombre de valeurs manquantes dans chaque colonne
missing_values = df.isna().sum()

# Afficher uniquement les colonnes qui ont des valeurs manquantes
print("Nombre de valeurs manquantes par colonne :")
print(missing_values[missing_values > 0])
