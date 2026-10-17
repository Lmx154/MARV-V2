// The X.25 CRC (CRC-16/MCRF4XX) MAVLink uses for its frame checksum.
export const X25_INIT = 0xffff;

export function x25Accumulate(crc: number, byte: number): number {
	let tmp = (byte ^ (crc & 0xff)) & 0xff;
	tmp = (tmp ^ (tmp << 4)) & 0xff;
	return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

export function x25(bytes: ArrayLike<number>, start = 0, end = bytes.length, crc = X25_INIT): number {
	for (let i = start; i < end; i++) crc = x25Accumulate(crc, bytes[i] as number);
	return crc;
}
