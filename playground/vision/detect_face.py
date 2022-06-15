import face_recognition
known_image = face_recognition.load_image_file("obama.jpeg")

face_locations = face_recognition.face_locations(known_image)

# Get the single face encoding out of elon-musk-1.jpg
face_location = face_locations[0]  # Only use the first detected face
face_encodings = face_recognition.face_encodings(known_image, [face_location])
elon_musk_knwon_face_encoding_1 = face_encodings[0]  # Pull out the one returned face encoding


# Load the image with unknown to compare
image = face_recognition.load_image_file("cover2.jpg")  # Load the image we are comparing
unknwon_face_encodings = face_recognition.face_encodings(image)

# Loop over each unknwon face encoding to see if the face matches either known encodings
print('Matches for elon-musk-in-group.jpg')
matches = []
for unknwon_face_encoding in unknwon_face_encodings:
    matches.append(face_recognition.compare_faces(
        [elon_musk_knwon_face_encoding_1],  # The known face encodings (can be only 1 - less is faster)
        unknwon_face_encoding  # The single unknown face encoding
    ))
    #print(matches)

print(matches)
face_locations = face_recognition.face_locations(image)
print(face_locations)


for i in range(len(matches)):
    if matches[i][0] == True:
        print("Image found at location: " + str(face_locations[i]))
        print()


'''
known_image = face_recognition.load_image_file("obama.jpeg")

unknown_image = face_recognition.load_image_file("cover2.jpg")

biden_encoding = face_recognition.face_encodings(known_image)[0]
unknown_encoding = face_recognition.face_encodings(unknown_image)[0]

results = face_recognition.compare_faces([biden_encoding], unknown_encoding)

face_locations = face_recognition.face_locations(unknown_image)

print(results)
print(face_locations)

'''