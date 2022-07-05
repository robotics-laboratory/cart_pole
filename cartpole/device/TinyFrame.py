import binascii

class TinyFrame:
    def __init__(self, peer:int=1):
        self.write = None # the writer function should be attached here

        self.id_listeners = {}
        self.type_listeners = {}
        self.fallback_listener = None
        self.peer = peer # the peer bit

        # ----------------------------- FRAME FORMAT ---------------------------------
        #  The format can be adjusted to fit your particular application needs

        #  If the connection is reliable, you can disable the SOF byte and checksums.
        #  That can save up to 9 bytes of overhead.

        # ,-----+-----+-----+------+------------+- - - -+-------------,
        # | SOF | ID  | LEN | TYPE | HEAD_CKSUM | DATA  | DATA_CKSUM  |
        # | 0-1 | 1-4 | 1-4 | 1-4  | 0-4        | ...   | 0-4         | <- size (bytes)
        # '-----+-----+-----+------+------------+- - - -+-------------'
        #
        # SOF ......... start of frame, usually 0x01 (optional, configurable)
        # ID  ......... the frame ID (MSb is the peer bit)
        # LEN ......... number of data bytes in the frame
        # TYPE ........ message type (used to run Type Listeners, pick any values you like)
        # HEAD_CKSUM .. header checksum
        #
        # DATA ........ LEN bytes of data (can be 0, in which case DATA_CKSUM is omitted as well)
        # DATA_CKSUM .. checksum, implemented as XOR of all preceding bytes in the message

        #  !!! BOTH SIDES MUST USE THE SAME SETTINGS !!!
        # Settings can be adjusted by setting the properties after init

        #  Adjust sizes as desired (1,2,4)
        self.ID_BYTES = 2
        self.LEN_BYTES = 2
        self.TYPE_BYTES = 1

        #  Checksum type
        # ('none', 'xor', 'crc16, 'crc32'
        self.CKSUM_TYPE = 'xor'

        #  Use a SOF byte to mark the start of a frame
        self.USE_SOF_BYTE = True
        #  Value of the SOF byte (if TF_USE_SOF_BYTE == 1)
        self.SOF_BYTE = 0x01

        self.next_frame_id = 0

        self.reset_parser()

        self._CKSUM_BYTES = None # will be updated on first compose / accept

    def reset_parser(self):
        # parser state: SOF, ID, LEN, TYPE, HCK, PLD, PCK
        self.ps = 'SOF'
        # buffer for receiving bytes
        self.rbuf = None
        # expected number of bytes to receive
        self.rlen = 0
        # buffer for payload or checksum
        self.rpayload = None
        # received frame
        self.rf = TF_Msg()

    def _reflect(self, num, width):
        """Reverts bit order of the given number

        Args:
            num (int): Number that should be reflected
            width (int): Size of the number in bits
        """
        reflected = 0

        for i in range(width):
            if (num >> i) & 1 != 0:
                reflected |= 1 << (width - 1 - i)

        return reflected

    def _crc16(self, data: bytes):
        xor_in = 0x0000  # initial value
        xor_out = 0x0000  # final XOR value
        poly = 0x8005  # generator polinom (normal form)

        reg = xor_in
        for byte in data:
            cur_byte = self._reflect(byte, 8)
            for i in range(8):
                topbit = reg & 0x8000
                if cur_byte & (0x80 >> i):
                    topbit ^= 0x8000
                reg <<= 1
                if topbit:
                    reg ^= poly
            reg &= 0xFFFF
        return self._reflect(reg ^ xor_out, 16)

    def _calc_cksum_bytes(self):
        if self.CKSUM_TYPE == 'none' or self.CKSUM_TYPE is None:
            return 0
        elif self.CKSUM_TYPE == 'xor':
            return 1
        elif self.CKSUM_TYPE == 'crc16':
            return 2
        elif self.CKSUM_TYPE == 'crc32':
            return 4
        else:
            raise Exception("Bad cksum type!")

    def _cksum(self, buffer) -> int:
        if self.CKSUM_TYPE == 'none' or self.CKSUM_TYPE is None:
            return 0

        elif self.CKSUM_TYPE == 'xor':
            acc = 0
            for b in buffer:
                acc ^= b
            return (~acc) & ((1<<(self._CKSUM_BYTES*8))-1)

        elif self.CKSUM_TYPE == 'crc16':
            return self._crc16(buffer)

        elif self.CKSUM_TYPE == 'crc32':
            return binascii.crc32(buffer)

        else:
            raise Exception("Bad cksum type!")

    def _gen_frame_id(self) -> int:
        """
        Get a new frame ID
        """

        frame_id = self.next_frame_id

        self.next_frame_id += 1
        if self.next_frame_id > ((1<<(8*self.ID_BYTES-1))-1):
            self.next_frame_id = 0

        if self.peer == 1:
            frame_id |= 1<<(8*self.ID_BYTES-1)

        return frame_id

    def _pack(self, num:int, bytes:int) -> bytes:
        """ Pack a number for a TF field """
        return num.to_bytes(bytes, byteorder='big', signed=False)

    def _unpack(self, buf) -> int:
        """ Unpack a number from a TF field """
        return int.from_bytes(buf, byteorder='big', signed=False)

    def query(self, type:int, listener, pld=None, id:int=None):
        """ Send a query """
        (id, buf) = self._compose(type=type, pld=pld, id=id)

        if listener is not None:
            self.add_id_listener(id, listener)

        self.write(buf)

    def send(self, type:int, pld=None, id:int=None):
        """ Like query, but with no listener """
        self.query(type=type, pld=pld, id=id, listener=None)

    def _compose(self, type:int, pld=None, id:int=None) -> tuple:
        """
        Compose a frame.

        frame_id can be an ID of an existing session, None for a new session.
        """
        if self._CKSUM_BYTES is None:
            self._CKSUM_BYTES = self._calc_cksum_bytes()

        if pld is None:
            pld = bytearray()

        if id is None:
            id = self._gen_frame_id()

        buf = bytearray()
        if self.USE_SOF_BYTE:
            buf.extend(self._pack(self.SOF_BYTE, 1))

        buf.extend(self._pack(id, self.ID_BYTES))
        buf.extend(self._pack(len(pld), self.LEN_BYTES))
        buf.extend(self._pack(type, self.TYPE_BYTES))

        if self._CKSUM_BYTES > 0:
            buf.extend(self._pack(self._cksum(buf), self._CKSUM_BYTES))

        if len(pld) > 0:
            buf.extend(pld)
            buf.extend(self._pack(self._cksum(pld), self._CKSUM_BYTES))

        return (id, buf)

    def accept(self, bytes):
        """
        Parse bytes received on the serial port
        """
        for b in bytes:
            self.accept_byte(b)

    def accept_byte(self, b:int):
        # TODO this seems ripe for rewrite to avoid repetitive code

        if self._CKSUM_BYTES is None:
            self._CKSUM_BYTES = self._calc_cksum_bytes()

        if self.ps == 'SOF':
            if self.USE_SOF_BYTE:
                if b != self.SOF_BYTE:
                    return

                self.rpayload = bytearray()
                self.rpayload.append(b)

            self.ps = 'ID'
            self.rlen = self.ID_BYTES
            self.rbuf = bytearray()

            if self.USE_SOF_BYTE:
                return

        if self.ps == 'ID':
            self.rpayload.append(b)
            self.rbuf.append(b)

            if len(self.rbuf) == self.rlen:
                self.rf.id = self._unpack(self.rbuf)

                self.ps = 'LEN'
                self.rlen = self.LEN_BYTES
                self.rbuf = bytearray()
            return

        if self.ps == 'LEN':
            self.rpayload.append(b)
            self.rbuf.append(b)

            if len(self.rbuf) == self.rlen:
                self.rf.len = self._unpack(self.rbuf)

                self.ps = 'TYPE'
                self.rlen = self.TYPE_BYTES
                self.rbuf = bytearray()
            return

        if self.ps == 'TYPE':
            self.rpayload.append(b)
            self.rbuf.append(b)

            if len(self.rbuf) == self.rlen:
                self.rf.type = self._unpack(self.rbuf)

                if self._CKSUM_BYTES > 0:
                    self.ps = 'HCK'
                    self.rlen = self._CKSUM_BYTES
                    self.rbuf = bytearray()
                else:
                    self.ps = 'PLD'
                    self.rlen = self.rf.len
                    self.rbuf = bytearray()
            return

        if self.ps == 'HCK':
            self.rbuf.append(b)

            if len(self.rbuf) == self.rlen:
                hck = self._unpack(self.rbuf)
                actual = self._cksum(self.rpayload)

                if hck != actual:
                    self.reset_parser()
                else:
                    if self.rf.len == 0:
                        self.handle_rx_frame()
                        self.reset_parser()
                    else:
                        self.ps = 'PLD'
                        self.rlen = self.rf.len
                        self.rbuf = bytearray()
                        self.rpayload = bytearray()
            return

        if self.ps == 'PLD':
            self.rpayload.append(b)
            self.rbuf.append(b)

            if len(self.rbuf) == self.rlen:
                self.rf.data = self.rpayload

                if self._CKSUM_BYTES > 0:
                    self.ps = 'PCK'
                    self.rlen = self._CKSUM_BYTES
                    self.rbuf = bytearray()
                else:
                    self.handle_rx_frame()
                    self.reset_parser()
            return

        if self.ps == 'PCK':
            self.rbuf.append(b)

            if len(self.rbuf) == self.rlen:
                pck = self._unpack(self.rbuf)
                actual = self._cksum(self.rpayload)

                if pck != actual:
                    self.reset_parser()
                else:
                    self.handle_rx_frame()
                    self.reset_parser()
            return

    def handle_rx_frame(self):
        frame = self.rf
        if frame.id in self.id_listeners and self.id_listeners[frame.id] is not None:
            lst = self.id_listeners[frame.id]
            rv = lst['fn'](self, frame)
            if rv == TF.CLOSE or rv is None:
                self.id_listeners[frame.id] = None
                return
            elif rv == TF.RENEW:
                lst.age = 0
                return
            elif rv == TF.STAY:
                return
            # TF.NEXT lets another handler process it

        if frame.type in self.type_listeners and self.type_listeners[frame.type] is not None:
            lst = self.type_listeners[frame.type]
            rv = lst['fn'](self, frame)
            if rv == TF.CLOSE:
                self.type_listeners[frame.type] = None
                return
            elif rv != TF.NEXT:
                return

        if self.fallback_listener is not None:
            lst = self.fallback_listener
            rv = lst['fn'](self, frame)
            if rv == TF.CLOSE:
                self.fallback_listener = None

    def add_id_listener(self, id:int, lst, lifetime:float=None):
        """
        Add a ID listener that expires in "lifetime" seconds

        listener function takes two arguments:
        tinyframe instance and a msg object
        """
        self.id_listeners[id] = {
            'fn': lst,
            'lifetime': lifetime,
            'age': 0,
        }

    def add_type_listener(self, type:int, lst):
        """
        Add a type listener

        listener function takes two arguments:
        tinyframe instance and a msg object
        """
        self.type_listeners[type] = {
            'fn': lst,
        }

    def add_fallback_listener(self, lst):
        """
        Add a fallback listener

        listener function takes two arguments:
        tinyframe instance and a msg object
        """
        self.fallback_listener = {
            'fn': lst,
        }

class TF_Msg:
    """ A TF message object """
    def __init__(self):
        self.data = bytearray()
        self.len = 0
        self.type = 0
        self.id = 0

    def __str__(self):
        return f"ID {self.id:X}h, type {self.type:X}h, len {self.len:d}, body: {self.data}"

class TF:
    """ Constants """
    STAY = 'STAY'
    RENEW = 'RENEW'
    CLOSE = 'CLOSE'
    NEXT = 'NEXT'
