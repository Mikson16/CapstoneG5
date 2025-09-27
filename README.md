# 📂 CapstoneG5 – Guía para trabajar con GitHub

Este documento explica cómo clonar y trabajar con este repositorio en **Linux**, **Windows** o **macOS**, usando **SSH** (forma más simple y sin contraseñas).

---

## 🔑 Paso 1: Crear clave SSH
Cada persona del grupo debe crear su propia clave SSH en su computador.

### Linux / macOS
1. Abrir la terminal.
2. Ejecutar:
   ```bash
   ssh-keygen -t ed25519 -C "tu_email@ejemplo.com"
Presiona Enter a todo (no pongas contraseña).

Ver tu clave pública:

```bash
cat ~/.ssh/id_ed25519.pub
```
**Windows**

1. Instalar Git for Windows.

2. Abrir Git Bash.

Ejecutar:

```bash
ssh-keygen -t ed25519 -C "tu_email@ejemplo.com"
```
Presiona Enter a todo.

Ver tu clave pública:

```bash
cat ~/.ssh/id_ed25519.pub
```

## 🔑 Paso 2: Registrar clave en GitHub
Copiar todo el texto que salió del comando anterior (empieza con ssh-ed25519).

Ir a GitHub → Settings → SSH and GPG keys.

Click en New SSH key.

Pegar la clave, darle un nombre (ej: "Laptop" o "PC Windows").

## 🔑 Paso 3: Probar conexión
En la terminal (Linux, macOS o Git Bash en Windows):

```bash
ssh -T git@github.com
```
Si funciona, saldrá un mensaje como:

```rust
Hi Tu_nick_name! You've successfully authenticated...
```
## 📥 Paso 4: Clonar el repositorio
Una sola vez por computador:

```bash
git clone git@github.com:Mikson16/CapstoneG5.git
cd CapstoneG5
```
## 📌 Paso 5: Flujo de trabajo básico
Cada vez que quieras trabajar:

Traer cambios nuevos (antes de empezar):

```bash
git pull
```
Guardar tus cambios:

```bash
git add .
git commit -m "Mensaje de lo que hiciste"
```
Enviar cambios al repositorio:

```bash
git push
```
## ⚠️ Notas importantes
No compartir la clave privada (id_ed25519). Solo se sube la pública a GitHub.

Siempre hacer git pull antes de empezar a trabajar, así evitamos conflictos.

Si hay un error de permisos, revisar que clonaron usando la URL SSH:

```scss
git@github.com:Mikson16/CapstoneG5.git
```
