import base64
import re
def decode_base64(data, altchars=b'+/'):
    """Decode base64, padding being optional.

    :param data: Base64 data as an ASCII byte string
    :returns: The decoded byte string.

    """
    data = re.sub(rb'[^a-zA-Z0-9%s]+' % altchars, b'', data)  # normalize
    missing_padding = len(data) % 4
    if missing_padding:
        data += b'=' * (4 - missing_padding)
    return base64.b64decode(data, altchars)

if __name__ == "__main__":
    data =  """
    load(344);
    run(12.0) ;
    """

    # Standard Base64 Encoding
    encodedBytes = base64.b64encode(data.encode("utf-8"))
    encodedStr = str(encodedBytes, "utf-8")

    res = bytes(encodedStr, 'utf-8')
    decodeBytes = decode_base64(res)
    decodeStr = str(decodeBytes, "utf-8")

    print("data:", data)
    print("encodedBytes:", encodedBytes)
    print("encodedStr:", encodedStr)
    print("decodeBytes:", decodeBytes)
    print("decodeStr:", decodeStr)

