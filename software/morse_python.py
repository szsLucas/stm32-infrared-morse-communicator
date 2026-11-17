"""
Comunicador Morse via Serial — Interface Gráfica
=================================================
Conecta ao STM32F401RCT6 via UART (115200 baud).

Fluxo TX: texto digitado → Morse → '\n' → UART → STM32 → IR LED
Fluxo RX: IR LED → STM32 → UART → Morse string → texto exibido

Dependências:
    pip install customtkinter pyserial
"""

import customtkinter as ctk
import serial
import threading
import unicodedata
import time

# ── Aparência ──────────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ── Dicionários Morse ──────────────────────────────────────────────────────────
MORSE_DICT = {
    'A': '.-',   'B': '-...', 'C': '-.-.', 'D': '-..',  'E': '.',
    'F': '..-.', 'G': '--.',  'H': '....', 'I': '..',   'J': '.---',
    'K': '-.-',  'L': '.-..', 'M': '--',   'N': '-.',   'O': '---',
    'P': '.--.', 'Q': '--.-', 'R': '.-.',  'S': '...',  'T': '-',
    'U': '..-',  'V': '...-', 'W': '.--',  'X': '-..-', 'Y': '-.--',
    'Z': '--..',
    '1': '.----', '2': '..---', '3': '...--', '4': '....-', '5': '.....',
    '6': '-....', '7': '--...', '8': '---..', '9': '----.', '0': '-----',
    ',': '--..--', '.': '.-.-.-', '?': '..--..', '/': '-..-.',
    '-': '-....-', '(': '-.--.', ')': '-.--.-',
}

REVERSO_MORSE_DICT = {v: k for k, v in MORSE_DICT.items()}

# ── Funções de Tradução ────────────────────────────────────────────────────────

def normalizar_texto(texto: str) -> str:
    """Remove acentos e converte para maiúsculas."""
    texto = unicodedata.normalize('NFKD', texto).encode('ASCII', 'ignore').decode('utf-8')
    return texto.upper()


def texto_para_morse(texto: str) -> str:
    """
    Converte texto em string Morse.
    Espaços entre palavras viram '/'.
    Exemplo: 'OLA MUNDO' → '--- .-.. .- / -- ..- -. -.. ---'
    """
    texto = normalizar_texto(texto)
    partes = []
    for letra in texto:
        if letra == ' ':
            partes.append('/')
        else:
            codigo = MORSE_DICT.get(letra)
            if codigo:
                partes.append(codigo)
            # Caracteres desconhecidos são silenciosamente ignorados
    return ' '.join(partes)


def morse_para_texto(morse: str) -> str:
    """
    Converte string Morse recebida do STM32 em texto.
    '/' (com espaço ao redor) representa separador de palavras.
    Exemplo: '--- .-.. .- / -- ..- -. -.. ---' → 'OLA MUNDO'
    """
    resultado = ''
    # Separa em tokens por espaço
    tokens = morse.strip().split(' ')
    for token in tokens:
        token = token.strip()
        if not token:
            continue
        if token == '/':
            resultado += ' '
        else:
            resultado += REVERSO_MORSE_DICT.get(token, '?')
    return resultado


# ── Interface Gráfica ──────────────────────────────────────────────────────────

class ComunicadorMorse(ctk.CTk):
    # Baud rate deve ser IGUAL ao definido no firmware (UART_BAUDRATE = 115200)
    BAUD_RATE = 115200

    def __init__(self):
        super().__init__()
        self.title("Comunicador Serial Morse — STM32F401RCT6")
        self.geometry("520x580")
        self.resizable(False, False)

        self.ser: serial.Serial | None = None
        self.conectado = False
        self._lock_serial = threading.Lock()  # Protege acesso concorrente à porta serial

        self._criar_widgets()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _criar_widgets(self):
        # Título
        ctk.CTkLabel(
            self,
            text="Terminal de Comunicação Morse",
            font=("Arial", 18, "bold")
        ).pack(pady=(14, 4))

        # Frame de conexão
        frame_conn = ctk.CTkFrame(self, fg_color="transparent")
        frame_conn.pack(pady=4, padx=20, fill="x")

        self.porta_entry = ctk.CTkEntry(
            frame_conn,
            placeholder_text="Porta (ex: COM3 ou /dev/ttyUSB0)",
            width=200
        )
        self.porta_entry.pack(side="left", padx=(0, 10))

        self.btn_conectar = ctk.CTkButton(
            frame_conn,
            text="Conectar",
            command=self._alternar_conexao
        )
        self.btn_conectar.pack(side="left", expand=True, fill="x")

        self.label_status = ctk.CTkLabel(
            self,
            text=f"Desconectado  •  {self.BAUD_RATE} baud",
            text_color="gray",
            font=("Arial", 12)
        )
        self.label_status.pack(pady=(2, 6))

        # Caixa de log
        self.log_box = ctk.CTkTextbox(self, height=300, state="disabled")
        self.log_box.pack(pady=6, padx=20, fill="both", expand=True)

        # Frame de envio
        frame_envio = ctk.CTkFrame(self, fg_color="transparent")
        frame_envio.pack(pady=10, padx=20, fill="x")

        self.msg_entry = ctk.CTkEntry(
            frame_envio,
            placeholder_text="Digite a mensagem em Português e pressione Enter"
        )
        self.msg_entry.pack(side="left", fill="x", expand=True, padx=(0, 10))
        self.msg_entry.bind("<Return>", lambda _e: self._enviar_mensagem())

        self.btn_enviar = ctk.CTkButton(
            frame_envio,
            text="Enviar",
            width=90,
            command=self._enviar_mensagem
        )
        self.btn_enviar.pack(side="right")

    # ── Conexão ───────────────────────────────────────────────────────────────

    def _alternar_conexao(self):
        if not self.conectado:
            self._conectar()
        else:
            self._desconectar()

    def _conectar(self):
        porta = self.porta_entry.get().strip()
        if not porta:
            self._log("[Sistema] Informe o nome da porta antes de conectar.")
            return

        try:
            self.ser = serial.Serial(
                porta,
                baudrate=self.BAUD_RATE,  # CRÍTICO: igual ao firmware
                timeout=0.1               # Leitura não bloqueia indefinidamente
            )
            self.conectado = True

            self.btn_conectar.configure(
                text="Desconectar",
                fg_color="red",
                hover_color="darkred"
            )
            self.label_status.configure(
                text=f"Conectado em {porta}  •  {self.BAUD_RATE} baud",
                text_color="green"
            )
            self.porta_entry.configure(state="disabled")
            self._log(f"--- Conectado à porta {porta} ({self.BAUD_RATE} baud) ---")

            # Thread daemon: encerra automaticamente quando a janela fecha
            threading.Thread(target=self._loop_escuta, daemon=True).start()

        except serial.SerialException as e:
            self._log(f"[Erro] Não foi possível abrir {porta}: {e}")

    def _desconectar(self):
        self.conectado = False

        with self._lock_serial:
            if self.ser and self.ser.is_open:
                self.ser.close()

        self.btn_conectar.configure(
            text="Conectar",
            fg_color=["#3B8ED0", "#1F6AA5"],
            hover_color=["#36719F", "#144870"]
        )
        self.label_status.configure(
            text=f"Desconectado  •  {self.BAUD_RATE} baud",
            text_color="gray"
        )
        self.porta_entry.configure(state="normal")
        self._log("--- Desconectado ---")

    # ── Envio ─────────────────────────────────────────────────────────────────

    def _enviar_mensagem(self):
        if not self.conectado:
            self._log("[Sistema] Conecte-se a uma porta primeiro.")
            return

        texto = self.msg_entry.get().strip()
        if not texto:
            return

        morse_msg = texto_para_morse(texto)
        if not morse_msg:
            self._log("[Sistema] Nenhum caractere Morse válido na mensagem.")
            return

        try:
            with self._lock_serial:
                # Envia string Morse + '\n' (firmware aguarda '\n' ou '\r')
                self.ser.write((morse_msg + '\n').encode('utf-8'))

            self._log(f"Você enviou : {texto}")
            self._log(f"  → Morse   : {morse_msg}\n")
            self.msg_entry.delete(0, 'end')

        except serial.SerialException as e:
            self._log(f"[Erro ao enviar]: {e}")
            self._desconectar()

    # ── Recepção (thread) ──────────────────────────────────────────────────────

    def _loop_escuta(self):
        """
        Lê linhas Morse do STM32 em background.
        Cada linha termina com '\n' (enviado pelo firmware após END_GAP).
        """
        buffer = b""

        while self.conectado:
            try:
                with self._lock_serial:
                    if self.ser and self.ser.in_waiting > 0:
                        buffer += self.ser.read(self.ser.in_waiting)
                    else:
                        # Sem dados: dorme brevemente para não saturar CPU
                        pass

                # Processa linhas completas acumuladas no buffer
                while b'\n' in buffer:
                    linha_bytes, buffer = buffer.split(b'\n', 1)
                    morse_recebido = linha_bytes.decode('utf-8', errors='replace').strip()

                    if morse_recebido:
                        texto_traduzido = morse_para_texto(morse_recebido)
                        # Atualiza GUI na thread principal via after()
                        self.after(0, self._log,
                                   f"Recebido  : {texto_traduzido}\n"
                                   f"  → Morse : {morse_recebido}\n")

            except serial.SerialException as e:
                if self.conectado:
                    self.after(0, self._log, f"[Erro de leitura]: {e}")
                    self.after(0, self._desconectar)
                break

            time.sleep(0.02)  # 20 ms — reduz uso de CPU sem comprometer latência

    # ── Log ───────────────────────────────────────────────────────────────────

    def _log(self, mensagem: str):
        """Acrescenta linha ao log. Pode ser chamado de qualquer thread via after()."""
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"{mensagem}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")


# ── Entry Point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = ComunicadorMorse()
    app.mainloop()
