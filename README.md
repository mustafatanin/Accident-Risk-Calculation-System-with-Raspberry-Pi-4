# Accident-Risk-Calculation-System

•	ÖZET
Öncelikle çeşitli sensörler kullanarak bunlardan aldığımız veriyi işleyerek bize kaza riskini hesaplamasını planladığımız bir sistem oluşturmayı hedefledik. Bunun için 6 tane sensör/modül ve bir adet kamera(webcam) kullandık. Veriyi aldığımız sensör/modüllerin haricinde elde ettiğimiz risk verisini görsel bir çıktıya dönüştürmek için programlanabilir bir RGB led kullandık. Ve sesli uyarı almak için de bir adet buzzer kullanarak projenin ana şemasını oluşturduk.



•	GİRİŞ: Ben ne yapmak istedim
Burada esas yapmak istediğimiz daha önce yapılmamış bir şey tasarlamak oldu. İnternette hazır kaynak kodları olan malzemesini temin ettiğiniz takdirde çok rahatlıkla yapabileceğiniz bir proje yapmayı değil özgün bir ürün ortaya çıkarmayı hedefledik. Örneğin kullandığımız GPS modülü Raspberry için üretilmiş bir modül bile değil. Biz Arduino için üretilmiş bu sensörü internette bulunan çok az kaynağa rağmen çalıştırıp hız hesaplamayı başardık. Bunun dışında projenin temel gayesini açıklamak gerekirse projemizde 6 tane sensör/modül ile bir adet kamera(Webcam) kullanarak  risk hesaplamayı ve bu hesaplanan risk ile sürücüyü ikaz etmeyi hedefledik. Bunları yaparken de tamamen gerçek hayata uyarlanabilir şekilde yapmaya gayret gösterdik dolayısıyla da projemizi gerçek bir otomobil üzerinde uyguladık. Projemizi herhangi bir maket veya mock-up üzerinde uygulamayı doğru bulmadık. Olabildiğince gerçek hayatta kullanılabilir bir ürün ortaya çıkarmaya çalıştık. 


•	UYGULAMA: Neyi neyle nasıl yaptın, neler kullandın…
Evvela kullandığımız bütün sensör ve modülleri saymakla başlayalım

•	Gy-NEO6MV2 GPS Modülü – Uçuş Kontrol Sistemi
Bu modülü kullanarak birim zamanda kat ettiğimiz mesafeyi uydudan gelen veri ile hesaplayarak hız verisi elde etmeyi amaçladık. Bu modülü kullanırken satın aldığımız yerin bize bozuk modül göndermesi sebebiyle uzun süre modülü çalıştırmayı başaramadık. Daha sonra yaptığımız tetkikler neticesinde modülün bozuk olduğuna karar verdik. Yeni temin ettiğimiz GPS modülünün headerslarını lehimleyerek işlemi tamamladık. Bu sayede hız ölçebilir hale geldik ve risk hesaplaması için oluşturduğumuz algoritma içerisinde bu hız verisinden yararlandık. 

•	ADXL345 3 Eksen İvme Ölçer
Bu sensör sayesinde anlık G kuvvetini hesapladık. X ve Y yönlerinden alınan G kuvvetini hesaplayarak otomobilin yan ve dikey eksende gösterdiği ani fren ve yan yatmaları hesapladık. Bu sayede otomobil ani risk oluşturacak bir eylemde bulunduğunda risk çıktısını etkiler hale geldi. 

•	HC-SR04 Ultrasonik Mesafe Sensörü 
Bu sensörden iki tane aracın ön kısmına yerleştirerek öndeki araç ile takip mesafesini ölçmeyi, çok yakınlaşılması halinde risk çıktısını artırmasını sağladık. Burada bu sensörden iki tane kullanmamızın sebebi aracın önünün büyük olması ve bu sensörün daha minimalist ortamlarda kullanılacak şekilde tasarlanması sebebiyle daha doğru veri alabilmek için iki tane kullanmayı tercih ettik.

•	Ses Sensörü Kartı
Bunu kullanmamızın aslında çok basit bir sebebi var araç içinde oluşacak yüksek sesli ortamın sürücünün dikkatini dağıtacağını düşündüğümüzden bu sensör ile ortamdaki özellikle müzik sesini ölçümleyerek risk hesabımızda kullandık.

•	Işık Sensörü Kartı 
Bunu kullanma sebebimiz ise karşıdan gelen otomobilin uzun far kullanması halinde yüksek ışığa maruz kalan sürücünün yolu görmesi güçleşeceğinden riski artıracaktır. Bu sebeple risk çıktımızda ışık sensörü de kullandık. 

•	Yağmur Sensörü Kartı 
Yağmur yağması durumunda yerler ıslanacağından otomobilin yola tutunması güçleşecektir. Bu da ister istemez riski artıracaktır. Burada bu sensör ile yağmur yağıp yağmadığını ölçerek risk hesabımız içerisine dahil ettik. 

•	Kamera(Webcam)
Kamera ile OpenCV kütüphanesi kullanarak göz kapakçıklarının kapalı kalma süresini ölçen bir kod ile sürücünün yorgunluğunu hesapladık.

•	NeoPixel 8’li Adreslenebilir RGB Led Şerit
Burada bütün bu sensörleri kullanarak elde ettiğimiz risk çıktısını görsel olarak sürücüye bildirmek için bu şekilde adreslenebilir bir led kullandık. Risk her yüzde 12.5 artış gösterdiğinde şeritteki ledler sırasıyla yanar hale gelmekte. Bu sayede sayısal çıktıyı görsel çıktı haline getirerek sürücü tarafından anlaşılmasını kolaylaştırdık.

•	Buzzer
Buzzer kullanma amacımız risk çok yüksek noktalara ulaştığında sürücüyü sesli olarak da ikaz ederek uyarılmasını kuvvetlendirdik.

•	Jumper Kablo 
Veri iletimi ve güç için kullandık.

•	10K ¼ Ω Direnç 
Mesafe sensöründe kullandık.

•	Breadboard
Sensörlere güç vermek için ve ana iskeleti oluşturmak için kullandık. 

•	Risk Hesaplama Algoritması
Kendimizin yazdığı bir algoritma sayesinde bütün sensörlerden gelen veriyi belirli hesaplamalardan geçirerek yüzde cinsinden bir risk çıktısı aldık. Bu algoritma bütün sensörlerden aynı anda veri alıyor ve aynı anda bunu işliyor bu sayede sensör/modüllerin birbirleriyle senkron çalışması sağlanarak risk hesaplanıyor.
Örneğin ivmeölçerden aldığımız g kuvveti verisi, GPS modülünden aldığımız veri ile senkron çalışarak riskin katlanarak artmasını sağlıyor. Burada 6 sensör/modül ve kameradan gelen veriyi anlık olarak hesaplamaya tabi tutup, riski bu şekilde hesapladığını da belirtmek istiyoruz.
	

•	SONUÇ: Başarılı oldun mu?
Elbette tamamen doğru risk verisi hesaplamak için uzun bilimsel çalışmalar yürütmek gerekiyor. Ve her alanın kendi uzmanı olduğunu düşünüldüğünde bunu bizim yapabilmemiz pek mümkün değildi. Zira bu konuda uzmanlığa sahip değiliz. Amacımız burada bir prototip geliştirerek sistemin genel çalışma mantığını proje sahibine aktarmak oldu. Bu konuda da son derece başarılı olduğumuzu düşünüyoruz. 
