echo "Creando alias de udev para las tarjetas de Justina..."

# Archivo de reglas
RULES_FILE="/etc/udev/rules.d/99-usb-alias.rules"

# Crear el archivo (no necesita chmod 666)
sudo touch $RULES_FILE

# Escribir reglas correctas
sudo tee $RULES_FILE > /dev/null << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A900fDgq", SYMLINK+="justinaRightArm", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A7005LgE", SYMLINK+="justinaHead", MODE="0666"
EOF

# Recargar reglas
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Alias creados:"
echo "  /dev/justinaRightArm"
echo "  /dev/justinaHead"