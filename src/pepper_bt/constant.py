# This is the module fo constants
SCREAM_PAINTING_DESCRIBTION = """
The Scream is a composition created by Norwegian artist Edvard Munch in 1893. 
The agonized face in the painting has become one of the most iconic images of art,
seen as symbolizing the anxiety of the human condition.
"""

# 
class paint:
    def __init__(self, **kwds):
        self.__dict__.update(**kwds)



PAINTINGS = {
    'nighthawks': paint(name = 'Nighthawks',
                         artist = 'Edward Hopper',
                         year = '',
                         describtion = '''
                            An iconic portrayal of a late-night diner in an urban setting,
                            emphasizing urban loneliness and isolation.
                         ''',
                         further_info = '''
                           "Nighthawks" by Edward Hopper, painted in 1942, is an iconic American artwork depicting a late-night diner scene in a lonely urban setting.
                           The stark lighting and isolated figures in the diner symbolize urban alienation, reflecting the mood of post-war America.
                           Hopper's masterful use of light and shadow creates a sense of melancholy and solitude, making the painting a hallmark of 20th-century American art.
                         ''',
                         lower_x=170,
                         upper_x=530
                         ),

    'mona_lisa': paint(name = 'Mona Lisa',
                         artist = 'Leonardo da Vinci',
                         year = '',
                         describtion = '''
                            One of the most recognizable and celebrated portraits in the world, depicting a woman with a mysterious smile.
                            It's renowned for its detailed sfumato technique and enigmatic expression.''',
                         further_info = '''
                            The Mona Lisa, created by Leonardo da Vinci, is a Renaissance masterpiece renowned for its subtle smile and captivating gaze.
                            Painted between 1503 and 1506, the portrait embodies da Vinci's mastery of the sfumato technique,
                            blending light and shadow to create a lifelike and enigmatic expression. The subject,
                            believed to be Lisa Gherardini, exudes an aura of timeless elegance and intrigue,
                            making the painting an enduring symbol of artistry and human fascination.
                           ''',
                         lower_x=540,
                         upper_x=880
                         ),

   'starry_night': paint(name = 'Starry Night',
                         artist = 'Vincent van Gogh',
                         year = '',
                         describtion = '''
                            A masterpiece known for its swirling, vibrant depiction of a night sky.
                            Van Gogh's bold use of color and expressive brushstrokes are characteristic of this painting
                         ''',
                         further_info = '''
                           "Starry Night" by Vincent van Gogh, completed in 1889, is a vivid and swirling masterpiece illustrating a tumultuous night sky above 
                           a French asylum where van Gogh was a patient. The painting, notable for its bold use of color and expressive brushwork,
                           captures the artist's emotional turmoil and his fascination with celestial beauty amidst personal struggle. The iconic piece stands 
                           as a hallmark of van Gogh's unique style and lasting influence on modern art.
                         ''',
                         lower_x=890,
                         upper_x=1240
                         ),

   'the_persistence_of_memory': paint(name = 'The Persistence of Memory',
                         artist = 'Salvador Dali',
                         year = '',
                         describtion = '''
                            A surrealistic artwork featuring melting clocks draped over various objects,
                            symbolizing the relativity of time and the subconscious mind.
                         ''',
                         further_info = '''
"The Persistence of Memory" by Salvador Dali, painted in 1931, is a surrealistic artwork featuring melting clocks draped over surreal landscapes and objects. 
Dali's intent was to portray the relativity of time and the malleability of reality,
 inviting viewers to ponder the fluidity of time and the subconscious mind.
   The painting remains an enduring symbol of surrealism and Dali's avant-garde approach to art.
                         ''',
                         lower_x=1250,
                         upper_x=1600
                         ),

#Reserved
    'guernica': paint(name = 'Guernica',
                         artist = 'Pablo Picasso',
                         year = '',
                         describtion = '''
                            A powerful anti-war artwork depicting the horrors of the Spanish Civil War.
                            It's known for its chaotic and emotional composition.
                         ''',
                         further_info = '''
                         ''',
                         lower_x=100,
                         upper_x=200
                         ),

    'girl_with_a_pearl_earring': paint(name = 'Girl with a Pearl Earring',
                         artist = '',
                         year = '',
                         describtion = '''
                            A captivating portrait of a young woman with a large pearl earring, 
                            showcasing Vermeer's mastery of light and shadow.
                         ''',
                         further_info = '''
                         ''',
                         lower_x=100,
                         upper_x=1570
                         )


}



# JUDGEMNT_OF_CAMBYSES_PAINTING_DESCRIBTION = """
# Rubens painted a commissioned work depicting the story of Sisamnes, a corrupt judge in the 
# Persian empire during Cambyses' reign. Completed between 1622 and 1626, the painting portrayed 
# the moment when Cambyses, depicted in a dynamic pose, appointed Sisamnes' son, Otanes, as the new judge, 
# placing his father's flayed skin as a canopy over the judge's seat as a reminder of the consequences of corruption.
# """

JUDGEMNT_OF_CAMBYSES_PAINTING_DESCRIBTION = """
Rubens painted a commissioned work depicting the story of Sisamnes
"""

# deprecated 
SCREAM_PAINTING_URL = "https://upload.wikimedia.org/wikipedia/commons/f/f4/The_Scream.jpg"


scream_painting = {'name':"the_scream",'describtion':SCREAM_PAINTING_DESCRIBTION}
judgment_of_cambyses_painting = {'name':"judgment_of_cambyses",'describtion':JUDGEMNT_OF_CAMBYSES_PAINTING_DESCRIBTION}

PRESENTATION_HELLO = "Which art do you choose?"

FURTHER_INFORMATION = "Do you want further information about painting? "

YES_NO = "Please say Yes or No after beep sound?"

SCREAM_PAINTING_FURTHER = """
THIS IS FURTHER INFORMATION ABOUT THE SCHREAM PAINTING
"""

JUDGEMNT_OF_CAMBYSES_PAINTING_FURTHER = """
THIS IS FURTHER INFORMATION ABOUT THE JUDGEMENT OF CHABYSES PAINTING
"""

GET_USER_FEEDBACK = "Please Give me a rate between one to five!"

THANKS = "Thank you"