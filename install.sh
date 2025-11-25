# Script para crear un alias del brazo de Justina

echo "Creando un alias de udev para el brazo de Justina..."

# 1. Crear el archivo de reglas
sudo touch /etc/udev/rules.d/99-usb-alias.rules

# 2. Dar permisos temporales para editarlo
sudo chmod 666 /etc/udev/rules.d/99-usb-alias.rules

# 3. Agregar la línea de alias (usamos tee porque sudo echo no redirige correctamente)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A900fDgq", SYMLINK+="justinaRightArm"' | sudo tee -a /etc/udev/rules.d/99-usb-alias.rules > /dev/null
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A7005LgE", SYMLINK+="justinaHead"' | sudo tee -a /etc/udev/rules.d/99-usb-alias.rules > /dev/null

# 4. Recargar reglas de udev
sudo udevadm control --reload-rules
sudo udevadm trigger

# 5. Asignar permisos al alias del brazo
sudo chmod +777 /dev/justinaRightArm

echo "Alias creado y permisos asignados: /dev/justinaRightArm"
echo "Alias creado y permisos asignados: /dev/justinaHead"