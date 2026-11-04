import qrcode

content = """Mã sản phẩm: mmb-6769
Số lượng: 1000"""

# Cấu hình mã QR
qr = qrcode.QRCode(
    version=1, 
    error_correction=qrcode.constants.ERROR_CORRECT_M,  
    box_size=10, 
    border=4,  
)

qr.add_data(content)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")

output_filename = "qr.png"
img.save(output_filename)

print(f"Đã tạo thành công mã QR và lưu tại: {output_filename}")