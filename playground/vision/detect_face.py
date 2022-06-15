import face_recognition


face_img = face_recognition.load_image_file("face.jpg")

face_locations = face_recognition.face_locations(face_img)

# Get the single face encoding out of elon-musk-1.jpg
face_location = face_locations[0]  # Only use the first detected face
face_encodings = face_recognition.face_encodings(face_img, [face_location])
person_face_encoding = face_encodings[0]  # Pull out the one returned face encoding


# Load the image with unknown to compare
#image = face_recognition.load_image_file("cover2.jpg")  # Load the image we are comparing
stream_img = face_recognition.load_image_file("stream.jpg")
unknwon_face_encodings = face_recognition.face_encodings(stream_img)

# Loop over each unknwon face encoding to see if the face matches either known encodings
#print('Matches for elon-musk-in-group.jpg')
matches = []
for unknwon_face_encoding in unknwon_face_encodings:
    matches.append(face_recognition.compare_faces(
        [person_face_encoding],  # The known face encodings (can be only 1 - less is faster)
        unknwon_face_encoding  # The single unknown face encoding
    ))
    #print(matches)

print(matches)
face_locations = face_recognition.face_locations(stream_img)
#print(face_locations)

for i in range(len(matches)):
    if matches[i][0] == True:
        print("Image found at location: " + str(face_locations[i]))
        print()
