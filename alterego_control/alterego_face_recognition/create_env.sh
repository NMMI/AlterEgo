#!/bin/bash

# Nome dell'ambiente da creare
ENV_NAME="face_tracking_test"

# Percorso del file requirements.txt
REQUIREMENTS_FILE="requirements.txt"

# Controlla se il file requirements.txt esiste
if [ ! -f "$REQUIREMENTS_FILE" ]; then
  echo "Errore: $REQUIREMENTS_FILE non trovato."
  exit 1
fi

# Crea l'ambiente Conda
echo "Creazione dell'ambiente Conda: $ENV_NAME"
conda create --name "$ENV_NAME" --yes python=3.10

# Attiva l'ambiente
echo "Attivazione dell'ambiente: $ENV_NAME"
conda activate "$ENV_NAME"

# Installa i pacchetti da requirements.txt
echo "Installazione dei pacchetti da $REQUIREMENTS_FILE"
pip install -r "$REQUIREMENTS_FILE"

echo "Ambiente $ENV_NAME creato e configurato con successo."
